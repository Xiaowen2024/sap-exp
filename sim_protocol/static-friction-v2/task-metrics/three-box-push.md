# Three-box push — task-level metrics

This extension sits on top of the [Static-friction v2 shared protocol](../README.md). The common interface metrics explain *how* an outcome occurred; this file adds the condition-level question: did the middle box end in the state the test was designed to produce?

## Expected-outcome agreement

Before running a condition, assign one expected label:

- **retain**: the middle box should remain within its prescribed support-relative displacement and pose tolerance;
- **eject**: the applied loading is intended to push the middle box beyond the preregistered ejection displacement/retention boundary.

At the end of each repeat, derive the simulated label from the same fixed geometric criterion. The metric is then:

| Metric | Meaning |
|---|---|
| Expected-outcome agreement | For each condition and repeat, whether the final middle-box state matches its preregistered expected label: a box intended to be ejected is ejected; a box intended to be retained remains retained. |

The label is a task-level binary endpoint. It does not replace the interface telemetry: a retained box can still exhibit nonphysical creep or contact switching, and an ejected box can fail for an unintended normal-contact instability.

## Repeated-condition statistics

Run each physical condition for \(M\) repeats, varying a declared perturbation set such as initial pose, contact seed/order where available, and controller/noise seed. Treat **eject** as the positive class:

| Statistic | Definition |
|---|---|
| True positive (TP) | Expected eject, simulated eject. |
| False positive (FP) | Expected retain, simulated eject. |
| False negative (FN) | Expected eject, simulated retain. |
| True negative (TN) | Expected retain, simulated retain. |
| False-positive rate | \(\mathrm{FPR}=FP/(FP+TN)\): spurious ejection of a condition expected to hold. |
| False-negative rate | \(\mathrm{FNR}=FN/(FN+TP)\): failure to eject a condition expected to eject. |
| Balanced accuracy | \(1-(\mathrm{FPR}+\mathrm{FNR})/2\). This is preferred to raw accuracy if retain/eject conditions are imbalanced. |

Report the per-condition pass fraction \(\hat p\) with its denominator \(M\), the pooled confusion matrix, and a 95% binomial confidence interval (Wilson or bootstrap). Also report macro-averaged FPR/FNR across physical-condition groups, so a dense sweep around one boundary cannot dominate the overall number.

## Required pre-registration

For every condition, store its expected label, ejection/retention threshold, force profile, duration, initial-pose perturbation distribution, random seed, and frozen engine configuration before evaluating outcomes.
