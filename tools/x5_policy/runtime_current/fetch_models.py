"""Download pinned upstream model files; never deploy or access hardware."""
from pathlib import Path
import argparse,hashlib,json,urllib.request
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--destination',type=Path,required=True);a=p.parse_args()
m=json.loads((Path(__file__).parent/'source/policy/models/manifest.json').read_text())
a.destination.mkdir(parents=True,exist_ok=True)
for name,info in m['models'].items():
    dest=a.destination/name
    if dest.exists():
        raw=dest.read_bytes()
    else:
        url=m['repository'].replace('https://github.com/','https://raw.githubusercontent.com/')+'/'+m['commit']+'/python/'+name
        with urllib.request.urlopen(url,timeout=30) as response:raw=response.read(info['size_bytes']+1)
    if len(raw)!=info['size_bytes'] or hashlib.sha256(raw).hexdigest()!=info['sha256']:raise ValueError('Model hash/size differs: '+name)
    if not dest.exists():
        with dest.open('xb') as stream:stream.write(raw)
    print(name,info['sha256'])
