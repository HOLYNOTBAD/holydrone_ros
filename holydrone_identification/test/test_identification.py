import csv
import tempfile
import unittest
from pathlib import Path
import numpy as np
from holydrone_identification.response import predict, identify_response, segments, read_csv
from holydrone_identification.physics import identify_physics, identify_bench


class IdentificationTest(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.root=Path(self.temporary.name)
    def tearDown(self):self.temporary.cleanup()

    def response_file(self,name,seed,corrupt=False):
        rng=np.random.RandomState(seed); rows=[]; duration=3.
        truth=[]
        for axis,channel in enumerate(('roll','pitch','yaw','thrust')):
            params=(.04+.01*axis,.025,1.1 if axis<3 else 32.,.02 if axis<3 else -.2)
            truth.append(params)
            t=np.arange(0,duration,.02)
            levels=rng.choice([-1.,1.],6)
            u=levels[np.minimum((t/.5).astype(int),5)]*(.35 if axis<3 else .04)+(0 if axis<3 else .3)
            y=predict(t,u,params[2]*u[0]+params[3],*params)+rng.normal(0,.001,len(t))
            if corrupt:y+=2*np.sin(10*t)
            for i,stamp in enumerate(t):
                row=dict(t=stamp+axis*4.,segment=axis,channel=channel,valid=1,
                         cmd_wx=0.,cmd_wy=0.,cmd_wz=0.,cmd_thrust=.3,wx=0.,wy=0.,wz=0.,fz=9.81)
                row[('cmd_wx','cmd_wy','cmd_wz','cmd_thrust')[axis]]=u[i]
                row[('wx','wy','wz','fz')[axis]]=y[i];rows.append(row)
        path=self.root/name
        with path.open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        return path,truth

    def test_response_recovery_and_holdout(self):
        train,truth=self.response_file('train.csv',1);test,_=self.response_file('test.csv',2)
        report=identify_response(train,test)
        self.assertTrue(report['passed'])
        for row,expected in zip(report['channels'],truth):
            np.testing.assert_allclose([row['tau_s'],row['delay_s']],expected[:2],atol=.004)
            self.assertAlmostEqual(row['gain'],expected[2],delta=.05)
        bad,_=self.response_file('bad.csv',3,True)
        self.assertFalse(identify_response(train,bad)['passed'])

    def test_aborted_pulse_rejected(self):
        path,_=self.response_file('aborted.csv',1)
        data=read_csv(path)
        augmented=np.empty(len(data),dtype=data.dtype.descr+[('phase','U16')])
        for name in data.dtype.names:augmented[name]=data[name]
        augmented['phase']='pulse'
        augmented['phase'][np.flatnonzero(data['segment']==0)[-1]]='fault'
        with self.assertRaises(ValueError):segments(augmented,0)

    def test_motor_bench_estimate(self):
        t=np.arange(0,8,.02)
        u=np.repeat([300.,700.,400.,600.],100)
        omega=predict(t,u,u[0],.04,.02,1.,0.)
        path=self.root/'bench.csv'
        with path.open('w') as stream:
            writer=csv.writer(stream);writer.writerow(['t','command_rad_s','omega_rad_s','thrust_n','torque_nm'])
            for a,b,c in zip(t,u,omega):writer.writerow([a,b,c,5e-6*c*c,1e-7*c*c])
        report=identify_bench(path)
        self.assertFalse(report['passed'])  # no independent bench validation supplied
        self.assertAlmostEqual(report['motor_constant_n_per_rad_s_squared'],5e-6,delta=1e-9)
        self.assertAlmostEqual(report['speed_response']['tau_s'],.04,delta=.005)

    def test_reused_validation_rejected(self):
        train,_=self.response_file('train.csv',1)
        with self.assertRaises(ValueError):identify_response(train,train)

    def physics_file(self,name,phase=0.,hover=False):
        t=np.arange(0,12,.02); mass=1.2;kt=5e-6;kq=1e-7
        inertia=np.array([.01,.012,.018]);damping=np.array([.001,.002,.003])
        frequencies=np.array([.8,1.3,1.7]); angles=t[:,None]*frequencies+phase
        w=.2*np.sin(angles) if not hover else np.zeros((len(t),3))
        derivative=.2*frequencies*np.cos(angles) if not hover else np.zeros_like(w)
        torque=derivative*inertia+np.cross(w,w*inertia)+w*damping
        positions=np.array([[.1,.1,0],[-.1,-.1,0],[.1,-.1,0],[-.1,.1,0]])
        signs=np.array([1,1,-1,-1])
        allocation=np.array([np.ones(4)*kt,positions[:,1]*kt,-positions[:,0]*kt,signs*kq])
        squared=np.linalg.solve(allocation,np.column_stack([np.full(len(t),mass*9.81),torque]).T).T
        self.assertTrue((squared>0).all())
        rpm=np.sqrt(squared)*60/(2*np.pi)
        path=self.root/name; rows=[]
        for i,stamp in enumerate(t):
            row=dict(t=stamp,wx=w[i,0],wy=w[i,1],wz=w[i,2],fz=9.81,vx=0.,vy=0.,vz=0.)
            row.update({'rpm%d'%j:rpm[i,j] for j in range(4)});rows.append(row)
        with path.open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        survey=dict(mass_kg=mass,center_of_mass_m=[0,0,0],rotor_positions_m=positions.tolist(),
                    yaw_torque_signs=signs.tolist(),esc_indices=[0,1,2,3],motor_order_verified=True,imu_bias_z_m_s2=0.)
        return path,survey,inertia

    def test_physical_parameter_recovery(self):
        train,survey,inertia=self.physics_file('physics.csv')
        validation,_,_=self.physics_file('physics_val.csv',.4)
        report=identify_physics(train,validation,survey)
        self.assertTrue(report['passed'])
        np.testing.assert_allclose(report['inertia_diagonal_kg_m2'],inertia,rtol=.02)
        self.assertAlmostEqual(report['motor_constant_n_per_rad_s_squared'],5e-6,delta=1e-9)
        self.assertAlmostEqual(report['gazebo_moment_constant_m'],.02,delta=.001)

    def test_hover_cannot_identify_inertia(self):
        train,survey,_=self.physics_file('hover.csv',hover=True)
        validation,_,_=self.physics_file('moving.csv',.2)
        with self.assertRaises(ValueError):identify_physics(train,validation,survey)


if __name__=='__main__':unittest.main()
