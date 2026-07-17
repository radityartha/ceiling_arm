import csv, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams.update({'font.size':9,'font.family':'serif','axes.edgecolor':'#888888',
                 'axes.linewidth':0.7,'xtick.color':'#444','ytick.color':'#444',
                 'text.color':'#222','axes.labelcolor':'#222'})
CSVDIR='docs/e3_jalanB_snapshot_2026-07-17/csv'
ACC='#2166AC'; G1='#7C7C7C'; G2='#B4B4B4'; INK='#222222'; GRID='#DDDDDD'

def energies(name):
    rs=[r for r in csv.DictReader(open(f'{CSVDIR}/{name}')) if r.get('success')=='1']
    return [float(r['traj_energy']) for r in rs if r['traj_energy'] not in ('','nan')]

fig,axb=plt.subplots(1,1,figsize=(3.4,2.55))
fig.subplots_adjust(left=0.20,right=0.97,bottom=0.17,top=0.87)

modes=[('energy\n(ours)','e3b_energy_v2_clean.csv',ACC),
       ('nearest','e3b_nearest.csv',G1),
       ('random','e3b_random.csv',G2)]
data=[energies(f) for _,f,_ in modes]; cols=[c for *_,c in modes]
bp=axb.boxplot(data,widths=0.55,patch_artist=True,showfliers=False,
               medianprops=dict(color=INK,lw=1.3),
               whiskerprops=dict(color='#888',lw=0.9),capprops=dict(color='#888',lw=0.9),
               boxprops=dict(lw=0))
for patch,c in zip(bp['boxes'],cols):
    patch.set_facecolor(c); patch.set_alpha(0.9)
means=[np.mean(d) for d in data]
axb.scatter(range(1,4),means,marker='D',s=26,color='white',edgecolor=INK,lw=0.9,zorder=5)
for i,m in enumerate(means):
    axb.text(i+1,m,f' {m:.1f}',ha='left',va='center',fontsize=8,color=INK)
axb.set_xticks([1,2,3]); axb.set_xticklabels([m for m,_,_ in modes])
axb.set_ylabel('trajectory energy (J)')
axb.set_title('measured energy per policy  ($\\diamond$ = mean)',fontsize=9,loc='left',color=INK)
axb.yaxis.grid(True,color=GRID,lw=0.6,zorder=0); axb.set_axisbelow(True)
for s in ('top','right'): axb.spines[s].set_visible(False)

fig.savefig('docs/IEEE-conference-Energy_Aware/figure/results_energy_only.pdf',bbox_inches='tight')
fig.savefig('/tmp/fig5_energy_only_preview.png',dpi=140,bbox_inches='tight')
print('means energy/nearest/random =', [round(m,2) for m in means])
print('saved figure/results_energy_only.pdf')
