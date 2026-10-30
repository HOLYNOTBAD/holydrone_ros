"""Physical identification with explicit scale, motor-order and excitation checks."""
import hashlib
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from .response import read_csv, fit_channel, score


def solve(matrix, target, label):
    scale = np.linalg.norm(matrix,axis=0)
    if np.any(scale<1e-12):
        raise ValueError(label+': unexcited parameter; supply a measured value or collect richer data')
    normalized = matrix/scale
    condition = float(np.linalg.cond(normalized))
    if np.linalg.matrix_rank(normalized)<matrix.shape[1] or condition>1e4:
        raise ValueError(label+': parameters are not identifiable (rank/conditioning)')
    values = np.linalg.lstsq(normalized,target,rcond=None)[0]/scale
    return values, condition


def flight_arrays(data, survey):
    required = ['t','wx','wy','wz','fz','vx','vy','vz']+['rpm%d'%i for i in range(4)]
    if any(n not in data.dtype.names for n in required):
        raise ValueError('Flight CSV needs '+str(required))
    values = np.column_stack([data[n] for n in required])
    if not np.isfinite(values).all():
        raise ValueError('Missing/nonfinite telemetry, including RPM; cannot identify physics from normalized T alone')
    if 'rpm_valid' in data.dtype.names and not np.all(data['rpm_valid']==1):
        raise ValueError('ESC telemetry is stale or missing; trim to a contiguous valid flight interval')
    if 'flight_valid' in data.dtype.names and not np.all(data['flight_valid']==1):
        raise ValueError('Flight feedback missing/disarmed; trim to a contiguous valid flight interval')
    dt=np.diff(data['t']); step=float(np.median(dt))
    if dt.max()>step*1.5 or dt.min()<step*.5:
        raise ValueError('Resample physical flight logs to a uniform grid before differentiating rates')
    omega=np.column_stack([data[n] for n in ('wx','wy','wz')])
    window=min(11,len(data)//2*2-1)
    smooth=savgol_filter(omega,window,3,axis=0)
    derivative=savgol_filter(omega,window,3,deriv=1,delta=step,axis=0)
    rpm=np.column_stack([data['rpm%d'%i] for i in survey['esc_indices']])
    if np.any(rpm<=0):
        raise ValueError('Only airborne, spinning-motor samples are supported')
    squared=(rpm*2*np.pi/60.)**2
    force=survey['mass_kg']*(data['fz']-survey['imu_bias_z_m_s2'])
    speed=np.linalg.norm(np.column_stack([data[n] for n in ('vx','vy','vz')]),axis=1)
    steady=(speed<survey.get('max_speed_m_s',.5)) & (np.linalg.norm(smooth,axis=1)<.3)
    # Differentiation edges are not valid observations.
    keep=slice(window//2,-window//2)
    return squared[keep],force[keep],smooth[keep],derivative[keep],steady[keep]


def torque_system(squared, omega, derivative, kt, survey):
    positions=np.asarray(survey['rotor_positions_m'],float)-np.asarray(survey['center_of_mass_m'],float)
    signs=np.asarray(survey['yaw_torque_signs'],float)
    if positions.shape!=(4,3) or signs.shape!=(4,) or not np.all(np.isin(signs,[-1,1])):
        raise ValueError('Need four rotor positions and four BODY yaw torque signs (+/-1)')
    forces=kt*squared
    tx=forces@positions[:,1]; ty=-forces@positions[:,0]; q=squared@signs
    n=len(omega); matrix=np.zeros((n,3,7)); target=np.zeros((n,3))
    x,y,z=omega.T; dx,dy,dz=derivative.T
    matrix[:,0,0]=dx; matrix[:,0,1]=-y*z; matrix[:,0,2]=y*z; matrix[:,0,4]=x
    matrix[:,1,0]=x*z; matrix[:,1,1]=dy; matrix[:,1,2]=-x*z; matrix[:,1,5]=y
    matrix[:,2,0]=-x*y; matrix[:,2,1]=x*y; matrix[:,2,2]=dz
    matrix[:,2,3]=-q; matrix[:,2,6]=z
    target[:,0]=tx; target[:,1]=ty
    return matrix.reshape(-1,7),target.reshape(-1)


def identify_physics(train_path, validation_path, survey):
    if Path(train_path).read_bytes()==Path(validation_path).read_bytes():
        raise ValueError('Independent physical validation log required')
    if not survey.get('motor_order_verified') or sorted(survey.get('esc_indices',[]))!=list(range(4)):
        raise ValueError('Measure/verify ESC order and rotor yaw-torque signs before fitting')
    for key in ('mass_kg','center_of_mass_m','rotor_positions_m','yaw_torque_signs','imu_bias_z_m_s2'):
        if key not in survey:
            raise ValueError('Missing measured survey field: '+key)
    if not np.isfinite(survey['mass_kg']) or survey['mass_kg']<=0:
        raise ValueError('A measured positive mass is required to fix force scale')
    training=flight_arrays(read_csv(train_path),survey)
    validation=flight_arrays(read_csv(validation_path),survey)
    squared,force,omega,derivative,steady=training
    if steady.sum()<30:
        raise ValueError('Need >=30 low-speed, low-rate samples for common rotor thrust coefficient')
    x=squared.sum(axis=1)[steady]; y=force[steady]
    kt=float(x@y/(x@x))
    if kt<=0:
        raise ValueError('Invalid thrust coefficient: check IMU sign, bias and RPM units')
    matrix,target=torque_system(squared,omega,derivative,kt,survey)
    known=survey.get('inertia_diagonal_kg_m2')
    if known is None:
        values,condition=solve(matrix,target,'inertia/yaw-torque fit')
    else:
        known=np.asarray(known,float)
        if known.shape!=(3,):raise ValueError('Measured inertia needs three diagonal entries')
        rest,condition=solve(matrix[:,3:],target-matrix[:,:3]@known,'yaw-torque/damping fit')
        values=np.r_[known,rest]
    inertia,kq,damping=values[:3],values[3],values[4:]
    if np.any(inertia<=0) or np.any(2*inertia>inertia.sum()) or kq<=0 or np.any(damping<0):
        raise ValueError('Nonphysical fitted parameters; inspect alignment, excitation and diagonal-inertia assumption')
    s,f,w,d,mask=validation
    if mask.sum()<30:raise ValueError('Insufficient steady validation samples')
    force_error=float(np.sqrt(np.mean((kt*s[mask].sum(axis=1)-f[mask])**2)))
    a,b=torque_system(s,w,d,kt,survey)
    error=(a@values-b).reshape(-1,3)
    accel_rmse=np.sqrt(np.mean((error/inertia)**2,axis=0))
    passed=bool(force_error<survey['mass_kg']*9.81*.1 and np.all(accel_rmse<2.))
    return dict(passed=passed, provenance='Known mass and geometry; calibrated FLU IMU; mechanical rotor RPM',
                assumptions=['identical vertical rotors', 'diagonal inertia', 'rigid body',
                             'low-speed thrust calibration', 'no rotor gyroscopic or inflow terms'],
                measured=survey, motor_constant_n_per_rad_s_squared=kt,
                reaction_torque_constant_nm_per_rad_s_squared=float(kq),
                gazebo_moment_constant_m=float(kq/kt), inertia_diagonal_kg_m2=inertia.tolist(),
                angular_damping_nm_per_rad_s=damping.tolist(), condition_number=condition,
                validation=dict(force_rmse_n=force_error, angular_acceleration_rmse_rad_s2=accel_rmse.tolist()),
                training_sha256=hashlib.sha256(Path(train_path).read_bytes()).hexdigest(),
                validation_sha256=hashlib.sha256(Path(validation_path).read_bytes()).hexdigest(),
                limitation='Candidate aggregate inertial parameters, not a drop-in base_link inertia; sensors/rotors must be accounted for when editing SDF')


def identify_bench(path):
    data=read_csv(path)
    required=('command_rad_s','omega_rad_s','thrust_n','torque_nm')
    if any(n not in data.dtype.names for n in required):raise ValueError('Bench CSV needs '+str(required))
    array=np.column_stack([data[n] for n in required])
    if not np.isfinite(array).all() or np.any(array[:,:2]<0):raise ValueError('Invalid bench values')
    t=data['t']-data['t'][0]; u=data['command_rad_s']; y=data['omega_rad_s']
    if np.ptp(u)<100:raise ValueError('Insufficient motor-speed excitation')
    derivative=np.gradient(y,t)
    steady=(np.abs(derivative)<50)&(y>100)
    if steady.sum()<20:raise ValueError('Need >=20 settled motor samples for force/torque coefficients')
    x=y[steady]**2
    kt=float(x@data['thrust_n'][steady]/(x@x)); kq=float(x@np.abs(data['torque_nm'][steady])/(x@x))
    params,boundary=fit_channel([(t,u,y)],0)
    return dict(passed=False, motor_constant_n_per_rad_s_squared=kt,
                reaction_torque_constant_nm_per_rad_s_squared=kq, gazebo_moment_constant_m=kq/kt,
                speed_response=dict(zip(('tau_s','delay_s','gain','bias'),map(float,params))),
                fit=score([(t,u,y)],params), at_parameter_boundary=boundary,
                limitation='Bench estimate only; validate independently and test rise/fall separately; command_rad_s must be calibrated speed demand, not raw PWM')
