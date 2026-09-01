#!/bin/bash

# Exit immediately if any command exits with a non-zero status
set -e

# Check if a commit message argument was provided
if [ -z "$1" ]; then
    echo "❌ Error: Missing commit message."
    echo "Usage: $0 \"your commit message\""
    exit 1
fi

# Store the argument as the commit message
COMMIT_MESSAGE="$1"

# Automatically detect the current branch name (e.g., main or master)
CURRENT_BRANCH=$(git branch --show-current)

if [ -z "$CURRENT_BRANCH" ]; then
    echo "❌ Error: Not a git repository (or no commits yet)."
    exit 1
fi

echo "📦 Staging all changes..."
git add .

echo "💾 Committing changes with message: \"$COMMIT_MESSAGE\"..."
git commit -m "$COMMIT_MESSAGE"

echo "🚀 Pushing to origin/$CURRENT_BRANCH..."
git push origin "$CURRENT_BRANCH"

echo "✅ Successfully pushed to GitHub!"
