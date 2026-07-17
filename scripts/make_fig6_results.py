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

fig,(axa,axb)=plt.subplots(1,2,figsize=(7.0,2.55))
fig.subplots_adjust(left=0.09,right=0.985,bottom=0.17,top=0.86,wspace=0.42)

# --- (a) E1 workspace-gain: locked vs active reachable volume (res 0.05) ---
lock,act = 1.920, 7.899           # m^3 at res 0.05 (raw), gain 4.11x
axa.bar([0],[lock],width=0.55,color=G1,zorder=3,label='rail locked')
axa.bar([1],[act], width=0.55,color=ACC,zorder=3,label='rail active')
for x,v in [(0,lock),(1,act)]:
    axa.text(x,v+0.15,f'{v:.1f}',ha='center',va='bottom',fontsize=9,color=INK)
# gain annotation
axa.annotate('', xy=(1,act*0.5), xytext=(0,act*0.5),
             arrowprops=dict(arrowstyle='->',color='#555',lw=1.0))
axa.text(0.5,act*0.55,r'$4.1\times$',ha='center',va='bottom',fontsize=10,color=ACC,fontweight='bold')
axa.set_xticks([0,1]); axa.set_xticklabels(['rail\nlocked','rail\nactive'])
axa.set_ylabel(r'reachable volume (m$^3$)')
axa.set_ylim(0,act*1.18); axa.set_title('(a) workspace gain from the rail DOF',fontsize=9,loc='left',color=INK)
axa.yaxis.grid(True,color=GRID,lw=0.6,zorder=0); axa.set_axisbelow(True)
for s in ('top','right'): axa.spines[s].set_visible(False)

# --- (b) E3 traj_energy boxplot per multi-arm policy ---
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
# mean markers (the win is on the mean) + labels
means=[np.mean(d) for d in data]
axb.scatter(range(1,4),means,marker='D',s=26,color='white',edgecolor=INK,lw=0.9,zorder=5)
for i,m in enumerate(means):
    axb.text(i+1,m,f' {m:.1f}',ha='left',va='center',fontsize=8,color=INK)
axb.set_xticks([1,2,3]); axb.set_xticklabels([m for m,_,_ in modes])
axb.set_ylabel('trajectory energy (J)')
axb.set_title('(b) measured energy per policy  ($\\diamond$ = mean)',fontsize=9,loc='left',color=INK)
axb.yaxis.grid(True,color=GRID,lw=0.6,zorder=0); axb.set_axisbelow(True)
for s in ('top','right'): axb.spines[s].set_visible(False)

fig.savefig('docs/IEEE-conference-Energy_Aware/figure/results.pdf',bbox_inches='tight')
fig.savefig('/tmp/fig6_preview.png',dpi=140,bbox_inches='tight')
print('means energy/nearest/random =', [round(m,2) for m in means])
print('saved figure/results.pdf')
