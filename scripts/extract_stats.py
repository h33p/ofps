import pandas as pd
import numpy as np
import os
import math

def degs(v):
    return v * 180.0 / math.pi

def avg_err(f):
    return (
        degs(f['error'].mean()),
        degs(f['error_r'].mean()),
        degs(f['error_p'].mean()),
        degs(f['error_y'].mean())
    )

def avg_perf(f):
    return f[0].mean()

decoders = ["av", "rlof", "farneback"]
clips = ["1", "2", "3", "4", "1dyn", "2dyn", "3dyn", "4dyn"]

est_map = {
    "almeida_0": "Almeida-RANSAC",
    "almeida_1": "Almeida",
    "homography_2": "Homography-RANSAC",
    "homography_3": "Homography-LMedS",
    "multiview_4": "Multiview-RANSAC",
    "multiview_5": "Multiview-LMedS",
    "libmv_6": "Libmv-7pt",
    "libmv_7": "Libmv-8pt"
}

def split_stats_dir(d):
    for e in decoders:
        if d.startswith(e):
            c = d[len(e) + 1:]
            if c in clips:
                return e, c
            break

    return None, None

def extract_stats(p):
    p = p.rsplit('/', 1)
    d = p[0]

    if len(p) > 1:
        p = p[1]
    else:
        p = ""

    out = {}
    perf_out = {}

    for e in decoders:
        out[e] = {}
        perf_out[e] = {}

    for combo in os.listdir(d):
        decoder, clip = split_stats_dir(combo)

        if not decoder:
            continue

        if p in combo:
            edir = d + '/' + decoder + "_" + clip
            for f in os.listdir(edir):

                if not f.endswith(".csv"):
                    continue

                f = f[:-4]

                if f.startswith('perf_'):
                    perf_mode = True
                    estimator = f[5:-3]
                else:
                    perf_mode = False
                    estimator = f

                if not estimator in est_map:
                    continue

                fname = est_map[estimator]

                if not fname in out[decoder]:
                    out[decoder][fname] = {}
                if not fname in perf_out[decoder]:
                    perf_out[decoder][fname] = {}

                fp = edir + '/' + f + '.csv'

                if perf_mode:
                    csv = pd.read_csv(fp, header = None)
                    perf = avg_perf(csv)
                    perf_out[decoder][fname][clip] = perf
                else:
                    csv = pd.read_csv(fp, index_col=0)
                    err = avg_err(csv)[0]
                    out[decoder][fname][clip] = err


    return out, perf_out

out, perf_out = extract_stats('mvec-stats/report2/')

out_dir = 'docs/statistics/'

for prefix, out in [('err', out), ('perf', perf_out)]:
    for decoder in out:

        data = {}

        index = []

        for estimator in out[decoder]:
            d = []
            for clip in out[decoder][estimator]:
                if not clip in index:
                    index.append(clip)

        index = sorted(index)

        for estimator in out[decoder]:
            d = []
            for clip in index:
                d.append(out[decoder][estimator][clip])
            data[estimator] = d

        df = pd.DataFrame(data, index = index)
        df.to_csv(out_dir + prefix + '_' + decoder + '.csv')

# Aggregate performance results into per-estimator stats.

index = [decoder for decoder in perf_out]

estimators = set()

for decoder in index:
    for estimator in perf_out[decoder]:
        estimators.add(estimator)

data = {}

for decoder in index:
    for estimator in perf_out[decoder]:
        avg = np.average([time for time in perf_out[decoder][estimator].values()])
        if not estimator in data:
            data[estimator] = []
        data[estimator].append(avg)

    for estimator in estimators:
        if not estimator in perf_out[decoder]:
            data[estimator].append(None)

df = pd.DataFrame(data, index = index)
df.to_csv(out_dir + 'perf.csv')

#print(out)
#print(perf_out)
