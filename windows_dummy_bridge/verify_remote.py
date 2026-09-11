"""Server verification: only GET health/capabilities/state."""
import json
from urllib.request import urlopen
results = {}
for path in ('health', 'capabilities', 'state'):
    with urlopen('http://127.0.0.1:18765/' + path, timeout=8) as response:
        results[path] = json.load(response)
assert results['health']['protocol_version'] == 4
assert results['health']['mode'] == 'transport'
assert results['capabilities']['protocol_version'] == 4
print(json.dumps(results, indent=2))
