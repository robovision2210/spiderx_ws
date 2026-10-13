#!/usr/bin/env python3
"""Cross-run table for the Cloud phase-1 observation (evidence helper). python3 summarize_runs.py OB"""
import glob
import json
import os
import sys

OB = sys.argv[1]
rows = []
for run in ('run_01', 'run_02', 'run_03'):
    R = os.path.join(OB, run)
    obs = [json.load(open(p)) for p in sorted(glob.glob(R + '/observer/*/observation.json'))]
    pf = [o for o in obs if o['mode'] == 'preflight'][0]
    ob = [o for o in obs if o['mode'] == 'observe'][0]
    st = ob['statistics']
    an = json.load(open(R + '/analysis.json'))
    sc = json.load(open(R + '/captures/scan_check_seams_excluded.json'))
    cap = json.load(open(R + '/captures/capture.json'))
    fails = [r['id'] for r in an['rows'] if r['result'] is False]
    nm = [r['id'] for r in an['rows'] if r['result'] is None]
    js = an['facts'].get('joint_states_capture', {})
    rows.append({
        'run': run, 'preflight': pf['ready'], 'observe_ready': ob['ready'],
        'observe_wall_s': st['clock']['wall_span_s'], 'sim_s': st['clock']['sim_span_s'],
        'rtf': st['clock']['real_time_factor'],
        'z_mean_m': st['body']['z_mean_m'], 'z_range_m': st['body']['z_range_m'],
        'tilt_max_rad': st['body']['tilt_max_rad'], 'xy_drift_m': st['body']['xy_drift_m'],
        'att_max_m': st['attachment']['max_translation_m'],
        'att_rot_rad': st['attachment']['max_rotation_rad'],
        'spawn_dev_m': st['attachment']['max_spawn_deviation_m'],
        'link_dev_m': st['attachment']['max_link_deviation_m'],
        'pose_samples': st['pose_receipt']['count'], 'pose_gap_max_s': st['pose_receipt']['max_s'],
        'pose_gap_p999_s': st['pose_receipt']['p999_s'],
        'js_gap_max_wall_s': st['joint_state_receipt']['max_s'],
        'js_sim_gap_max_s': st['joint_state_sim_gap_max_s'],
        'sample_codes': st.get('sample_codes'), 'frame_ids': st.get('header_frame_ids'),
        'graph_start': {k: ob['graph']['start'][k] for k in ('command_publishers', 'action_clients',
                                                              'joint_state_publishers')},
        'graph_end': {k: ob['graph']['end'][k] for k in ('command_publishers', 'action_clients',
                                                          'joint_state_publishers')},
        'controllers_end': ob['controllers']['end'], 'goals_sent': ob.get('goals_sent'),
        'publishers_created': ob.get('publishers_created'),
        'capture_cmd_msgs': len(cap['command_messages']),
        'capture_goal_statuses': sum(len(a['statuses']) for a in cap['action_status']),
        'js_capture_max_change_rad': js.get('max_change_over_capture_rad'),
        'scan_fit_dx_dy_dyaw': sc['fit_minus_expected_m_m_rad'],
        'scan_fit_1sigma_robust': sc['fit_1sigma_robust_m_m_rad'],
        'scan_surface_medians_m': {k: v['median_residual_m'] for k, v in sc['surfaces'].items()},
        'scan_beams_used': sc['fit_beams_used'],
        'failed_rows': fails, 'not_measured_rows': nm,
    })
json.dump(rows, open(os.path.join(OB, 'summary_runs.json'), 'w'), indent=1)
for r in rows:
    print(json.dumps(r))
