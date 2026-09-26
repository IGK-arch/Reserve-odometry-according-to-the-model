import json
from collections import Counter
import numpy as np

from analyze_gnss import DATA, OUT, extract, enu


def main():
    summary=json.loads((OUT/'summaries.json').read_text())
    bags=[x for x in summary if x['master_path']]
    def where(x):
        return 'east' if x>-150 else 'west' if x<-4300 else 'middle'
    modes=Counter((where(x['master_path']['start_enu'][0]),where(x['master_path']['end_enu'][0])) for x in bags)
    print('start_end_modes',modes)
    full=[x for x in bags if x['master_path'].get('net_m',0)>4400]
    print('full_net_count',len(full),'full_raw_distance_quantiles',np.quantile([x['master_path']['length_m_clipped_steps'] for x in full],[0,.1,.5,.9,1]))
    print('stationary_short',[(x['bag'],x['master_path']['start_enu']) for x in bags if x['master_path'].get('length_m_clipped_steps',1e9)<5])
    for bag in ['30618_0e41eac3','30618_0f120b35','30639_584b6e32','30618_0686195f']:
        _,_,a=extract(DATA/bag/f'{bag}_0.db3')
        v=a['mv'];dt=np.diff(v[:,1]);sp=np.linalg.norm(v[:,2:5],axis=1)
        good=(dt>0)&(dt<.3)&(sp[:-1]<20)&(sp[1:]<20)
        distance=np.sum(.5*(sp[:-1]+sp[1:])[good]*dt[good])
        print(bag,'velocity_distance_m',distance,'velocity_max',sp.max(),'gnss_span_s',v[-1,1]-v[0,1])


if __name__=='__main__':main()
