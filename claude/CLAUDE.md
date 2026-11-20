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
- Never post to a PR without the user's explicit approval for that post. This
  covers reviews, inline comments, thread replies and issue comments. After a
  review, present the findings first (in any language, e.g. the English draft
  itself), and post only after the user approves. Approval of one post doesn't
  carry over to the next review.
- Always self-review the diff before creating a PR. Read the full diff against
  the base branch, check for unintended changes (unrelated files, lock churn,
  debug leftovers), confirm that docs/comments that reference the changed
  behavior are still accurate, and fix any issues before opening the PR.

## Code review and refactoring

When reviewing or refactoring a change, act as a senior engineer who will
maintain this repository long term. Judge not only whether the code works, but
whether other developers can safely understand, modify, and extend it for years.

This applies to every review, including a re-review after new commits come
back. Don't stop at checking that earlier comments were addressed: re-read the
whole change against its base with the full checklist below, because a fix
commit can introduce new redundancy (e.g. a module reduced to a one-line
forward) or architectural problems.

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

### Verifying review findings

Past mistakes came from stating unverified inferences as facts: a premise about
how code behaves that was never checked, an impact estimated without measuring
it, and a suggested fix whose side effects were never considered. Before
presenting findings:

1. **Back every finding with evidence**: a measured value, a command that
   reproduces it, or the exact code lines it rests on. If you cannot verify a
   claim, phrase it as a question, not as a defect. Don't state its severity
   either.
2. **Check what a suggested fix breaks.** Before proposing a fix, trace at least
   its main side effect, e.g. which data or callers the new behavior affects.
3. **Use a subagent as a falsifier, not as a second reviewer.** For non-trivial
   reviews, give a subagent each finding (claim plus evidence) and ask it to
   find reasons the claim does not hold. Resolve any disagreement by checking
   the code or running it yourself, not by letting the agents debate.
4. **State the scope in every review report**: how many of the PR's changed
   files you read in full (e.g. "19/19"), which tests ran and which were
   skipped, and what you could not verify. A "no findings" conclusion counts as
   a claim too. It requires the whole-diff read like any other.
5. **On a re-review, start from the base, not from your previous review.** Get
   the file list from `git diff --stat $(git merge-base origin/<base> <head>) <head>`
   (or the PR's file list), read every listed file in full, and write the scope
   line from that list, naming the range (e.g. "main...19fbb3d, 25/25 files").
   Never reuse an earlier round's scope line. Reading only the commits since the
   last review and reporting it as a whole-PR read once led to an approval based
   on a false scope.

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
