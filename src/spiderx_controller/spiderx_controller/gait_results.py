"""M4.5 OFFLINE result records shared by the kinematics, metrics and report modules.

A Check is one named pass/fail test of one gait, with the measured value, the threshold it was
compared with, and a list of the exact failures (leg, sample, value). Every check carries an
honesty label so that reports never present an approximation as a measurement:

    exact          property of the kinematic model (URDF + IK), evaluated at every sample
    sampled        evaluated on the discrete samples only (finer events between samples are missed)
    approximation  rests on simplifying assumptions (e.g. quasi-static, CAD masses, point feet)
    heuristic      a proxy for comparison only (e.g. "energy" proxies), NOT a physical measurement
"""

from dataclasses import dataclass, field

LABELS = ('exact', 'sampled', 'approximation', 'heuristic')
MAX_LISTED_FAILURES = 20


@dataclass
class Check:
    name: str
    passed: bool
    value: object                 # the measured quantity (worst case), or None if not computable
    threshold: object             # what it was compared with, or None
    unit: str
    label: str                    # one of LABELS
    description: str
    failures: list = field(default_factory=list)   # first MAX_LISTED_FAILURES failure strings
    failure_count: int = 0
    applicable: bool = True       # False: reported for information only, not part of the verdict

    def __post_init__(self):
        if self.label not in LABELS:
            raise ValueError(f'check {self.name}: label must be one of {LABELS}, got {self.label}')

    def add_failure(self, text):
        self.failure_count += 1
        if len(self.failures) < MAX_LISTED_FAILURES:
            self.failures.append(text)

    def as_dict(self):
        return {'name': self.name, 'passed': self.passed, 'applicable': self.applicable,
                'value': self.value, 'threshold': self.threshold, 'unit': self.unit,
                'label': self.label, 'description': self.description,
                'failure_count': self.failure_count, 'failures': list(self.failures)}


__all__ = ['Check', 'LABELS', 'MAX_LISTED_FAILURES']
