#!/usr/bin/env python3
"""Offline identification. Outputs candidates without modifying aircraft assets."""
import argparse
from pathlib import Path
import yaml
from holydrone_identification.response import identify_response
from holydrone_identification.physics import identify_physics, identify_bench


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='kind',required=True)
    for kind in ('response','physics','bench'):
        p=sub.add_parser(kind)
        p.add_argument('--train',type=Path,required=True)
        p.add_argument('--output',type=Path,required=True)
        if kind!='bench':p.add_argument('--validation',type=Path,required=True)
        if kind=='physics':p.add_argument('--survey',type=Path,required=True)
        if kind=='response':p.add_argument('--training-config',type=Path,help='Optional existing QUAD_RL configuration to update, only when validation passes')
    args=parser.parse_args()
    if args.output.exists():parser.error('Output already exists; use a new path')
    if args.kind=='response':
        report=identify_response(args.train,args.validation)
    elif args.kind=='physics':
        report=identify_physics(args.train,args.validation,yaml.safe_load(args.survey.read_text()))
    else:report=identify_bench(args.train)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:yaml.safe_dump(report,stream,sort_keys=False)
    if args.kind=='response' and args.training_config:
        if not report['passed']:
            raise SystemExit('Validation failed; report saved, training config was not generated')
        config=yaml.safe_load(args.training_config.read_text())
        if config.get('environment_type')!='bodyrate_thrust':
            raise SystemExit('Only bodyrate_thrust training configs are supported')
        config['env']['response_model']=report
        config['env']['allow_unvalidated_model']=False
        # Do not export a 3 rad/s training envelope from a 0.2 rad/s experiment.
        config['env']['rate_limits']=[min(abs(c['validated_input_range'][0]),abs(c['validated_input_range'][1]))
                                      for c in report['channels'][:3]]
        config['env']['thrust_limits']=report['channels'][3]['validated_input_range']
        config['experiment_note']='Locally identified response; action bounds restricted to independently tested inputs'
        with args.output.with_suffix('.training.yaml').open('x') as stream:yaml.safe_dump(config,stream,sort_keys=False)
    print('Report:',args.output,'passed:',report['passed'])


if __name__=='__main__':main()
