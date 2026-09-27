"""Verify production Release build against every frozen pure replay hash."""
from pathlib import Path
import argparse, hashlib, json, sys
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'evaluation'))
import navigation_gnss_stress as stress


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--exe', type=Path, required=True)
    p.add_argument('--inputs', type=Path, required=True)
    p.add_argument('--scratch', type=Path, required=True)
    args = p.parse_args()
    args.scratch.mkdir(parents=True, exist_ok=True)
    source = ROOT / 'evaluation/results/round4/pure_navigation_comparison.json'
    report = json.loads(source.read_text())
    rows = []
    binary_hash = stress.sha256(args.exe)
    for bag in report['bags']:
        for mode, previous in bag['modes'].items():
            input_path = args.inputs / f"{bag['bag']}_{mode}_input.csv"
            assert stress.sha256(input_path) == previous['input_sha256']
            output_path = args.scratch / 'output.csv'
            stress.replay(args.exe.resolve(), input_path, output_path, int(bag['bag'][:5]))
            actual = stress.sha256(output_path)
            expected = previous['output_sha256']['candidate']
            rows.append({'bag': bag['bag'], 'mode': mode, 'expected_sha256': expected,
                         'production_sha256': actual, 'identical': actual == expected})
    assert stress.sha256(args.exe) == binary_hash
    result = {'command': sys.argv, 'exe_sha256': binary_hash,
              'script_sha256': stress.sha256(Path(__file__)),
              'frozen_report_sha256': stress.sha256(source),
              'all_identical': all(r['identical'] for r in rows), 'comparisons': rows}
    (ROOT / 'evaluation/results/round4/integration_parity.json').write_text(
        json.dumps(result, indent=2) + '\n')
    print(len(rows), 'comparisons; all identical:', result['all_identical'])
    assert result['all_identical']


if __name__ == '__main__':
    main()
