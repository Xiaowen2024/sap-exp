"""Identical synchronized timing boundary for the two GPU playback adapters."""
import statistics
import time


def graph_rollouts(wp,device,graph,blocks,reset,check,repeats):
    if repeats<1:raise ValueError('at least one measured rollout is required')
    times=[];checks=[]
    for trial in range(repeats+1):
        reset();wp.synchronize_device(device)
        start=time.perf_counter()
        for _ in range(blocks):wp.capture_launch(graph)
        wp.synchronize_device(device)
        elapsed=time.perf_counter()-start
        row=check();row['trial']=trial;row['wall_s']=elapsed;checks.append(row);times.append(elapsed)
        print('GPU benchmark',trial,'warmup' if trial==0 else 'measured',elapsed,row,flush=True)
        if not row['finite'] or row.get('overflow') or row['steps']!=row['expected_steps']:raise RuntimeError('invalid benchmark rollout: '+str(row))
    return {'warmup_full_rollout_s':times[0],'measured_full_rollouts_s':times[1:],
        'median_wall_s':statistics.median(times[1:]),'min_wall_s':min(times[1:]),'max_wall_s':max(times[1:]),
        'repeats':repeats,'checks':checks,'scope':'one world, full trajectory; GPU controller + native collision/solve/integration + 10-ms state logging + health checks',
        'excluded':'reset, setup/JIT/capture, GPU-to-host readback, file I/O and rendering',
        'synchronization':'device synchronization immediately before and after timed CUDA-graph launches','cuda_graph_steps_per_launch':2}
