#!/usr/bin/env python3
"""Analyze baseline-vs-fish swarm experiment.

Reads results/<cond>/seed<k>/{odom_i.csv,cmd_i.csv,obstacles.csv}, computes
per-run metrics, aggregates mean+/-std across seeds, writes:
  results/metrics_per_run.csv
  results/summary_table.md  (+ summary_table.csv)
  results/fig_*.png
"""
import os, glob, math, csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

RES = os.environ.get("RES", "/out")
N = 7
CONDS = ["baseline", "fish"]
LABEL = {"baseline": "Baseline (quadrotor)", "fish": "Robotic fish"}
COL = {"baseline": "#1f77b4", "fish": "#ff7f0e"}

# nominal start / target formation (from normal_hexagon.launch)
INIT = np.array([[-26.0,0.0],[-23.4,-1.5],[-26.0,-3.0],[-28.6,-1.5],
                 [-28.6,1.5],[-26.0,3.0],[-23.4,1.5]])
TARGET = np.array([[26.0,0.0],[28.6,-1.5],[26.0,-3.0],[23.4,-1.5],
                   [23.4,1.5],[26.0,3.0],[28.6,1.5]])

# desired pairwise distances of the (rigid) formation template
def pairwise(P):
    d=[]
    for a in range(len(P)):
        for b in range(a+1,len(P)):
            d.append(np.linalg.norm(P[a]-P[b]))
    return np.array(d)
DSTAR = pairwise(INIT)


def load_csv(path):
    if not os.path.exists(path): return None
    a = np.genfromtxt(path, delimiter=",", names=True)
    if a.size == 0: return None
    return np.atleast_1d(a)


def run_metrics(cond, seed_dir):
    odom, cmd = {}, {}
    for i in range(N):
        odom[i] = load_csv(os.path.join(seed_dir, "odom_%d.csv"%i))
        cmd[i]  = load_csv(os.path.join(seed_dir, "cmd_%d.csv"%i))
    if any(odom[i] is None or len(odom[i])<10 for i in range(N)):
        return None
    obs = load_csv(os.path.join(seed_dir, "obstacles.csv"))

    # --- kinematics from cmd (commanded, dynamically-constrained) ---
    v_max=[]; v_mean=[]; a_max=[]; a_mean=[]; jerk_rms=[]
    for i in range(N):
        c = cmd[i]
        if c is None or len(c)<10:
            v_max.append(np.nan); v_mean.append(np.nan); a_max.append(np.nan)
            a_mean.append(np.nan); jerk_rms.append(np.nan); continue
        t = c["t"]
        sp = np.sqrt(c["vx"]**2 + c["vy"]**2 + c["vz"]**2)
        am = np.sqrt(c["ax"]**2 + c["ay"]**2 + c["az"]**2)
        moving = sp > 0.05
        v_max.append(np.nanmax(sp))
        v_mean.append(np.nanmean(sp[moving]) if moving.any() else 0.0)
        a_max.append(np.nanmax(am))
        a_mean.append(np.nanmean(am[moving]) if moving.any() else 0.0)
        # jerk = d(acc)/dt
        dt = np.diff(t); dt[dt<=0]=np.nan
        jx=np.diff(c["ax"])/dt; jy=np.diff(c["ay"])/dt; jz=np.diff(c["az"])/dt
        jm=np.sqrt(jx**2+jy**2+jz**2)
        jerk_rms.append(np.sqrt(np.nanmean(jm**2)))

    # --- path length + completion + clearance from odom ---
    path_len=[]; depart=[]; arrive=[]; complete=[]
    # common time grid for formation
    tmax = min(odom[i]["t"][-1] for i in range(N))
    tgrid = np.arange(0, tmax, 0.1)
    X = np.zeros((len(tgrid),N)); Y = np.zeros((len(tgrid),N))
    for i in range(N):
        o=odom[i]; t=o["t"]; x=o["x"]; y=o["y"]
        path_len.append(np.sum(np.sqrt(np.diff(x)**2+np.diff(y)**2+np.diff(o["z"])**2)))
        X[:,i]=np.interp(tgrid,t,x); Y[:,i]=np.interp(tgrid,t,y)
        # departure / arrival
        sp = np.sqrt(np.diff(x)**2+np.diff(y)**2)/np.maximum(np.diff(t),1e-3)
        mv = np.where(sp>0.1)[0]
        depart.append(t[mv[0]] if len(mv) else np.nan)
        dist_tgt = np.sqrt((x-TARGET[i,0])**2+(y-TARGET[i,1])**2)
        arr = np.where(dist_tgt<1.5)[0]
        arrive.append(t[arr[0]] if len(arr) else np.nan)
        complete.append((arrive[-1]-depart[-1]) if (len(arr) and len(mv)) else np.nan)
    final_dist = [math.hypot(odom[i]["x"][-1]-TARGET[i,0], odom[i]["y"][-1]-TARGET[i,1]) for i in range(N)]

    # --- formation RMSE over time ---
    frmse=[]
    for k in range(len(tgrid)):
        P=np.column_stack([X[k],Y[k]])
        D=pairwise(P)
        frmse.append(np.sqrt(np.mean((D-DSTAR)**2)))
    frmse=np.array(frmse)
    # inter-agent min distance over time
    inter_min=np.inf
    for k in range(len(tgrid)):
        P=np.column_stack([X[k],Y[k]])
        for a in range(N):
            for b in range(a+1,N):
                inter_min=min(inter_min,np.linalg.norm(P[a]-P[b]))

    # --- obstacle clearance (2D footprint via KD-tree) ---
    # EGO-Swarm deliberately grazes obstacles, so min-clearance ~0 is not
    # discriminative; we report MEAN clearance along the path and a near-miss
    # fraction (points within 0.3 m of an obstacle surface).
    clearance=np.nan; clearance_min=np.nan; near_miss=np.nan
    if obs is not None and len(obs)>0:
        oxy=np.column_stack([obs["x"],obs["y"]])
        key=np.round(oxy/0.1).astype(np.int64)
        _,uidx=np.unique(key,axis=0,return_index=True)
        tree=cKDTree(oxy[uidx])
        alld=[]
        for i in range(N):
            o=odom[i]
            pts=np.column_stack([o["x"][::3],o["y"][::3]])
            d,_=tree.query(pts)
            # only count points inside the obstacle field x-extent
            inside = (pts[:,0]>oxy[:,0].min()-1)&(pts[:,0]<oxy[:,0].max()+1)
            alld.append(d[inside])
        alld=np.concatenate(alld)
        clearance=float(np.mean(alld))
        clearance_min=float(np.min(alld))
        near_miss=float(np.mean(alld<0.3))

    arrived = sum(1 for d in final_dist if d<1.5)
    success = (arrived==N) and (inter_min>0.2)

    return dict(
        cond=cond,
        v_max=np.nanmean(v_max), v_mean=np.nanmean(v_mean),
        a_max=np.nanmean(a_max), a_mean=np.nanmean(a_mean),
        jerk_rms=np.nanmean(jerk_rms),
        path_len=np.nanmean(path_len),
        completion=np.nanmean(complete),
        form_rmse_mean=float(np.mean(frmse)), form_rmse_max=float(np.max(frmse)),
        inter_min=float(inter_min), clearance=clearance,
        clearance_min=clearance_min, near_miss=near_miss,
        arrived=arrived, success=int(success),
        _frmse=frmse, _tgrid=tgrid, _X=X, _Y=Y, _cmd=cmd, _obs=obs,
    )


def main():
    runs={c:[] for c in CONDS}
    detail={}
    for c in CONDS:
        for sd in sorted(glob.glob(os.path.join(RES,c,"seed*"))):
            m=run_metrics(c,sd)
            if m is None:
                print("skip (incomplete):",sd); continue
            seed=os.path.basename(sd)
            m["seed"]=seed
            runs[c].append(m)
            detail[(c,seed)]=m
            print("%-9s %-6s success=%d arrived=%d form_rmse=%.3f vmax=%.2f amax=%.2f clr=%.2f"%(
                c,seed,m["success"],m["arrived"],m["form_rmse_mean"],m["v_max"],m["a_max"],m["clearance"]))

    # per-run CSV
    keys=["cond","seed","success","arrived","v_max","v_mean","a_max","a_mean",
          "jerk_rms","path_len","completion","form_rmse_mean","form_rmse_max",
          "inter_min","clearance","clearance_min","near_miss"]
    with open(os.path.join(RES,"metrics_per_run.csv"),"w",newline="") as f:
        w=csv.writer(f); w.writerow(keys)
        for c in CONDS:
            for m in runs[c]:
                w.writerow([m[k] for k in keys])

    # aggregate mean +/- std
    metrics=[("v_max","Max speed","m/s"),("v_mean","Mean speed","m/s"),
             ("a_max","Max accel","m/s^2"),("a_mean","Mean accel","m/s^2"),
             ("jerk_rms","RMS jerk","m/s^3"),("path_len","Path length","m"),
             ("completion","Traversal time","s"),("form_rmse_mean","Formation RMSE (mean)","m"),
             ("form_rmse_max","Formation RMSE (max)","m"),("inter_min","Min inter-agent dist","m"),
             ("clearance","Mean obstacle clearance","m"),("near_miss","Near-miss fraction (<0.3 m)","-")]
    def agg(c,k):
        v=np.array([m[k] for m in runs[c]],dtype=float)
        return np.nanmean(v),np.nanstd(v)

    lines=["# Baseline vs Robotic-fish: results","",
           "Identical EGO-Swarm formation planner, identical random-forest environment "
           "(7 agents, hexagon formation, %d obstacle seeds). Only the dynamic-feasibility "
           "envelope differs: **baseline** max_vel=1.5 m/s, max_acc=8.0 m/s^2; "
           "**fish** max_vel=1.0 m/s, max_acc=2.5 m/s^2."%max(len(runs['baseline']),len(runs['fish'])),
           "",
           "| Metric | Unit | %s | %s |"%(LABEL['baseline'],LABEL['fish']),
           "|---|---|---|---|"]
    for k,name,unit in metrics:
        bm,bs=agg("baseline",k); fm,fs=agg("fish",k)
        lines.append("| %s | %s | %.3f ± %.3f | %.3f ± %.3f |"%(name,unit,bm,bs,fm,fs))
    # success rate
    bsr=100*np.mean([m["success"] for m in runs["baseline"]]) if runs["baseline"] else 0
    fsr=100*np.mean([m["success"] for m in runs["fish"]]) if runs["fish"] else 0
    lines.append("| Success rate | %% | %.0f | %.0f |"%(bsr,fsr))
    lines.append("")
    lines.append("_n = %d seeds per condition. ± is std across seeds._"%len(runs["baseline"]))
    md="\n".join(lines)+"\n"
    open(os.path.join(RES,"summary_table.md"),"w").write(md)
    # csv version
    with open(os.path.join(RES,"summary_table.csv"),"w",newline="") as f:
        w=csv.writer(f); w.writerow(["metric","unit","baseline_mean","baseline_std","fish_mean","fish_std"])
        for k,name,unit in metrics:
            bm,bs=agg("baseline",k); fm,fs=agg("fish",k)
            w.writerow([name,unit,bm,bs,fm,fs])
    print("\n"+md)

    # ---------------- figures ----------------
    _figures(detail, runs)
    print("wrote figures + tables to", RES)


def _figures(detail, runs):
    # Fig 1: trajectories seed1 baseline vs fish
    fig,axes=plt.subplots(1,2,figsize=(13,4.5),sharex=True,sharey=True,
                          constrained_layout=True)
    for ax,c in zip(axes,CONDS):
        m=detail.get((c,"seed1")) or (runs[c][0] if runs[c] else None)
        if m is None: continue
        if m["_obs"] is not None:
            ax.scatter(m["_obs"]["x"],m["_obs"]["y"],s=0.4,c="0.7",alpha=0.5,linewidths=0)
        cols=plt.cm.tab10.colors
        for i in range(N):
            ax.plot(m["_X"][:,i],m["_Y"][:,i],"-",color=cols[i%10],lw=1.4)
            ax.plot(m["_X"][0,i],m["_Y"][0,i],"o",color=cols[i%10],ms=5)
            ax.plot(m["_X"][-1,i],m["_Y"][-1,i],"s",color=cols[i%10],ms=5)
        ax.set_title("%s  (seed=%s)"%(LABEL[c],m.get("seed","?")))
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_ylim(-8,8); ax.grid(True,alpha=0.3)
    fig.suptitle("Top-down trajectories through the random forest (o=start, s=end)")
    fig.savefig(os.path.join(RES,"fig_trajectories.png"),dpi=130); plt.close(fig)

    # Fig 2: speed & accel profile (seed1, mean across agents vs time)
    fig,axes=plt.subplots(1,2,figsize=(13,4.2))
    for c in CONDS:
        m=detail.get((c,"seed1")) or (runs[c][0] if runs[c] else None)
        if m is None: continue
        # mean speed/accel across agents over a common grid from cmd
        tg=np.arange(0,20,0.05); SP=[]; AM=[]
        for i in range(N):
            cc=m["_cmd"][i]
            if cc is None: continue
            sp=np.sqrt(cc["vx"]**2+cc["vy"]**2+cc["vz"]**2)
            am=np.sqrt(cc["ax"]**2+cc["ay"]**2+cc["az"]**2)
            SP.append(np.interp(tg,cc["t"],sp)); AM.append(np.interp(tg,cc["t"],am))
        if SP:
            axes[0].plot(tg,np.mean(SP,axis=0),color=COL[c],label=LABEL[c])
            axes[1].plot(tg,np.mean(AM,axis=0),color=COL[c],label=LABEL[c])
    axes[0].axhline(1.5,ls="--",color=COL["baseline"],alpha=0.5)
    axes[0].axhline(1.0,ls="--",color=COL["fish"],alpha=0.5)
    axes[1].axhline(8.0,ls="--",color=COL["baseline"],alpha=0.5)
    axes[1].axhline(2.5,ls="--",color=COL["fish"],alpha=0.5)
    axes[0].set_title("Commanded speed (mean over agents)"); axes[0].set_xlabel("t [s]"); axes[0].set_ylabel("|v| [m/s]")
    axes[1].set_title("Commanded acceleration (mean over agents)"); axes[1].set_xlabel("t [s]"); axes[1].set_ylabel("|a| [m/s$^2$]")
    for a in axes: a.grid(True,alpha=0.3); a.legend()
    fig.tight_layout(); fig.savefig(os.path.join(RES,"fig_kinematics.png"),dpi=130); plt.close(fig)

    # Fig 3: formation RMSE vs time (seed1)
    fig,ax=plt.subplots(figsize=(8,4))
    for c in CONDS:
        m=detail.get((c,"seed1")) or (runs[c][0] if runs[c] else None)
        if m is None: continue
        ax.plot(m["_tgrid"],m["_frmse"],color=COL[c],label=LABEL[c])
    ax.set_title("Formation error (RMSE of pairwise distances vs template)")
    ax.set_xlabel("t [s]"); ax.set_ylabel("formation RMSE [m]"); ax.grid(True,alpha=0.3); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(RES,"fig_formation.png"),dpi=130); plt.close(fig)

    # Fig 4: bar comparison of key aggregate metrics. Traversal time (tens of
    # seconds) is plotted on its own panel so it doesn't flatten the sub-2
    # kinematic/quality bars that share the other panel's scale.
    small_keys=[("v_max","Max speed\n[m/s]"),("a_max","Max accel\n[m/s²]"),
                ("jerk_rms","RMS jerk\n[m/s³]"),("form_rmse_mean","Form. RMSE\n[m]"),
                ("clearance","Mean clearance\n[m]")]
    time_keys=[("completion","Traversal\n[s]")]
    fig,(ax,axt)=plt.subplots(1,2,figsize=(12,4.5),
                              gridspec_kw={"width_ratios":[5,1]},
                              constrained_layout=True)
    def grouped(a,keys):
        x=np.arange(len(keys)); w=0.38
        for j,c in enumerate(CONDS):
            means=[np.nanmean([mm[k] for mm in runs[c]]) for k,_ in keys]
            stds=[np.nanstd([mm[k] for mm in runs[c]]) for k,_ in keys]
            a.bar(x+(j-0.5)*w,means,w,yerr=stds,capsize=3,label=LABEL[c],color=COL[c])
        a.set_xticks(x); a.set_xticklabels([n for _,n in keys])
        a.grid(True,axis="y",alpha=0.3)
    grouped(ax,small_keys); grouped(axt,time_keys)
    ax.legend()
    fig.suptitle("Aggregate metric comparison (mean ± std across seeds)")
    fig.savefig(os.path.join(RES,"fig_bars.png"),dpi=130); plt.close(fig)


if __name__=="__main__":
    main()
