# Global conventions

## Pull requests

- Write everything that ends up on a PR in English, in every repository. This
  applies regardless of the language used in the conversation or in the
  repository's own documents. It covers:
  - PR titles and descriptions (body)
  - review bodies and inline review comments
  - replies on review threads, and PR issue comments
- Replies to the user in the conversation stay in the user's language; only the
  text posted to GitHub is English.
- Always self-review the diff before creating a PR. Read the full diff against
  the base branch, check for unintended changes (unrelated files, lock churn,
  debug leftovers), confirm that docs/comments that reference the changed
  behavior are still accurate, and fix any issues before opening the PR.

## Code review and refactoring

When reviewing or refactoring a change, act as a senior engineer who will
maintain this repository long term. Judge not only whether the code works, but
whether other developers can safely understand, modify, and extend it for years.

### What to check

1. **Simplicity & readability**
   - Prefer the simplest, most intuitive implementation a first-time reader can follow.
   - Avoid over-abstraction, unneeded design patterns, and needless splitting into classes/functions.
   - Don't generalize single-use logic into a framework without a reason.
   - Explain non-obvious design intent in comments; favor clarity over brevity.
2. **Maintainability & extensibility**
   - Separate parts likely to change from stable parts.
   - Avoid coupling to a specific dataset, model, environment, or config value; remove magic numbers and hard-coding.
   - Keep needed extensibility, but don't build features ahead of need (YAGNI).
   - Design so new features require minimal changes to existing code.
3. **Architecture & consistency**
   - Understand the repository's structure and design before changing it.
   - Follow existing naming, style, directory layout, and API conventions.
   - Reuse existing code that already does the same thing.
   - Don't add unnecessary dependencies or libraries; keep module responsibilities clear and coupling low.
   - Avoid unrelated edits and large reformatting.
4. **Code quality**
   - Remove duplication, unnecessary processing, and dead code.
   - Handle errors properly; never swallow exceptions in a way that hides problems.
   - Make types, interfaces, and function inputs/outputs explicit.
   - Revisit logging, debug code, and temporary workarounds.
   - Keep code testable; don't add excessive defensive code or unjustified fallbacks.
5. **PR scope & reviewability**
   - Keep PRs at a size and structure a human can review.
   - Exclude changes unrelated to the PR's purpose; split independent changes.
   - Prefer the minimal change that achieves the goal over wholesale rewrites.
   - Be able to justify every change; for large diffs, consider splitting by responsibility or feature.

### Process

1. **Understand** the purpose of the change, the existing architecture, and the responsibilities of the touched code (diff the current branch against the base branch).
2. **Review critically** for readability, maintainability, complexity, extensibility, consistency, and scope.
3. **Reconsider the design**: is this complexity actually needed, or is there a simpler way?
4. **Refactor** where needed while preserving existing behavior and external interfaces.
5. **Verify** with tests and static analysis that behavior did not change unintentionally.
6. **Final review**: confirm a maintainer reading the PR for the first time can easily understand the design and the changes.

First summarize the review findings and the improvement plan, then make the necessary fixes.
If an unclear point matters for a decision, ask instead of changing the spec on a guess.

### Principles

- **Simplicity over cleverness.**
- **Maintainability over short-term convenience.**
- **YAGNI**: don't add features just because they might be needed.
- **DRY, but not at the expense of clarity.**
- **KISS**: choose the simplest design that solves the problem.
- **Minimal diff**: keep the change as small as the goal allows.

### Final checklist

- Can a developer seeing this code for the first time understand it?
- Can another developer change the spec easily six months from now?
- Could a simpler design achieve the same result?
- Were any unnecessary abstractions or dependencies added?
- Is the PR too large for its purpose?
- Would I accept maintaining this code myself for the next several years?

Evaluate code — including your own — strictly from a third-party perspective rather
than justifying the existing implementation. But don't do large refactors just for
the sake of improvement: keep existing functionality and external interfaces, and
keep the scope of changes to the necessary minimum.
