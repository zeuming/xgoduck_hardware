"""Record exact source edits against immutable baseline inputs, offline only."""
import argparse,difflib,hashlib,json
from pathlib import Path

p=argparse.ArgumentParser(description=__doc__)
for name in ('baseline','firmware-baseline','candidate','recipe','patch'):
 p.add_argument('--'+name,type=Path,required=True)
a=p.parse_args();assert not a.recipe.exists() and not a.patch.exists()
sha=lambda b:hashlib.sha256(b).hexdigest()
base=json.loads((a.baseline/'snapshot.json').read_text())
changes=[];patch=[]
for path in sorted(a.candidate.rglob('*')):
 if not path.is_file() or any(x in path.parts for x in ('.pio','__pycache__')):continue
 rel=path.relative_to(a.candidate)
 if path.suffix not in ('.py','.cpp','.h','.ini','.js','.html','.csv'):continue
 old=(a.firmware_baseline/Path(*rel.parts[1:]) if rel.parts[0]=='firmware' else a.baseline/rel)
 before=old.read_bytes() if old.exists() else None;after=path.read_bytes()
 if before==after:continue
 left=(before or b'').decode().splitlines(keepends=True);right=after.decode().splitlines(keepends=True)
 edits=[dict(start=i,end=j,lines=right[k:l]) for op,i,j,k,l in difflib.SequenceMatcher(None,left,right,autojunk=False).get_opcodes() if op!='equal']
 check=list(left)
 for edit in reversed(edits):check[edit['start']:edit['end']]=edit['lines']
 assert ''.join(check).encode()==after
 changes.append(dict(path=rel.as_posix(),before_sha256=sha(before) if before is not None else None,after_sha256=sha(after),edits=edits))
 patch.extend(difflib.unified_diff(left,right,fromfile='baseline/'+rel.as_posix(),tofile='candidate/'+rel.as_posix()))
spec=dict(schema=1,baseline_installation_sha256=base['installation_sha256'],upstream_commit='8cdbbd84710d856581982c9eaf0d5e2970666232',changes=changes)
a.recipe.write_text(json.dumps(spec,indent=2)+'\n');a.patch.write_text(''.join(patch))
print(json.dumps(dict(changed_files=len(changes),recipe_sha256=sha(a.recipe.read_bytes()),hardware_access=False)))
