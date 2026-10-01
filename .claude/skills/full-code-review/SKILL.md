---
name: full-code-review
description: Review a completed repository as a whole after change-level acceptance, including an orphan-interface scan.
---

Read `prompts/full-code-review.md` and follow it. Do not modify active OpenSpec artifacts or replace independent Evaluation.

## Mandatory step: orphan-interface scan

This skill always includes an orphan-interface scan with its own report section. Run it even when the rest of the review is scoped narrowly.

Start from `prompts/full-code-review.md` for the scan criteria and report shape. In short: find production interfaces, entrypoints, configuration keys, protocol shapes and build/install surfaces that have no real consumer, then report each one with a concrete removal-or-replan recommendation. A caller that exists only in a test is not a consumer.

The scan is read-only. Report findings; never delete, deprecate or re-plan an interface yourself. Removing a surface that has an approved requirement needs a Planner decision.
