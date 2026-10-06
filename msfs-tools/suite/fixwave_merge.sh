#!/bin/bash
REPO="D:/A380/fbw-xp-worktrees/fs2020-672384b"
WT="D:/A380/fbw-wt"
BR="${BR:-fixwave}"
ORDER="${ORDER:-overbroad verify-elecmode verify-area17 verify-area6-a verify-area6-b verify-area2 verify-area3-4-5-11 verify-area7-8-10-15-19-20 verify-fbw slats ecam-24 ecam-28 ecam-21 ecam-36 ecam-32 ecam-72-73 ecam-76-77-79 ecam-29 ecam-34 ecam-misc}"
for P in $ORDER; do
  W="$WT/$BR-$P"
  if [ -n "$(git -C "$W" status --porcelain)" ]; then
    git -C "$W" add -A && git -C "$W" commit -q -m "$BR $P: changes left uncommitted at the deadline" && echo "$P: committed leftovers"
  fi
  N=$(git -C "$REPO" rev-list --count HEAD..$BR/$P)
  if [ "$N" = "0" ]; then echo "$P: nothing to merge"; continue; fi
  if git -C "$REPO" merge --no-ff --no-edit -q "$BR/$P" -m "merge $BR/$P" 2>/dev/null; then
    echo "$P: merged ($N commits, $(git -C "$REPO" diff --stat HEAD^1 HEAD | tail -1))"
  else
    echo "$P: CONFLICT in: $(git -C "$REPO" diff --name-only --diff-filter=U | tr '\n' ' ')"
    exit 1
  fi
done
echo "all merged"
