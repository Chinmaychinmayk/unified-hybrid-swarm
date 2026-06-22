# Demo videos

Demo clips for the YouTube upload and the journal supplementary material. They
are stored as **GIF** so they render inline on GitHub and inside the paper.

| File | Source | Shows |
| --- | --- | --- |
| `01_sim_hexagon_formation.gif` | `simulation/fig/normal_hexagon.gif` | Seven-agent hexagon school holding formation. |
| `02_sim_hexagon_forest_navigation.gif` | `simulation/fig/normal_hexagon_2.gif` | The school weaving through the random-forest obstacle field. |
| `03_sim_set_goal_navigation.gif` | `simulation/fig/set_goal_normal_hexagon.gif` | Interactive "2D Nav Goal" target selection in RViz. |
| `04_experiment_baseline_vs_fish_playback.gif` | `simulation/experiment/results/figs_rviz/swarm_playback.gif` | Side-by-side baseline-quadrotor vs robotic-fish playback (seed 1). |

## Converting to MP4 for YouTube

YouTube prefers MP4 (H.264). With `ffmpeg` installed:

```bash
for f in *.gif; do
  ffmpeg -i "$f" -movflags +faststart -pix_fmt yuv420p \
    -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" "${f%.gif}.mp4"
done
```

To stitch them into a single reel:

```bash
printf "file '%s'\n" *.mp4 > list.txt
ffmpeg -f concat -safe 0 -i list.txt -c copy combined_demo.mp4
```
