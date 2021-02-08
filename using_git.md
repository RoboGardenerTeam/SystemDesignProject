# Our Git Workflow

Our git workflow is basically GitHub flow, as [described here](https://guides.github.com/introduction/flow/). I highly recommend taking a look at the link, it takes only 5 minutes to read.

### Workflow steps

Essentially the workflow is as follows:

1. You are on the `main` branch, and would like to work on a feature.
2. Instead of working directly on `main`, you create a **feature branch** to do all your work on.
    - Create and checkout a **new** feature branch (to simply switch to a branch that already exists, omit the `-b` flag).
        - `$ git checkout -b feature_branch_name`
    - At this point our feature branch is only a local branch (exists only our local machine). We wish to have this feature branch live on the shared repository as well so that others can work on it too.
        - `$ git push -u origin feature_branch_name`
            - The `-u` flag is to make the local feature branch a tracking branch, read more [here](https://git-scm.com/book/en/v2/Git-Branching-Remote-Branches). Don't forget to use this flag!
        - This pushes our local feature branch to the shared repository.
        - At this point the feature branch is a **shared** branch, do **NOT** make destructive changes (e.g. force pushing or rebasing).
3. Do all your development work on the `feature_branch_name` branch.
    - This is your standard `git add`, `git commit`, and `git push`.
4. You feel like your work on the feature branch is ready to be merged into `main`. To do this, we open a **pull request**.
    - On the GitHub repo webpage, click the pull requests tab.
    - Click the big green button that says _New pull request_.
    - Make sure the `base` branch we would like to merge into is `main` (this could be another feature branch if you really want though).
    - Make sure the `compare` branch is the `feature_branch_name` branch.
    - Click the big green button that says _Create pull request_.
    - Leave a short description of the changes that you made while on `feature_branch_name`. This is so that the pull request reviewers will have an easy time knowing the changes that were made.
    - Again, click the big green button that says _Create pull request_.
5. Other team members review the pull request and decide if it's ready to be merged or needs a bit more work.
    - Once the pull request is created, other team members will take a look at the changes made and review it to see if everything is ok, and potentially add comments for suggestions and improvements.
    - If `feature_branch_name` still needs a bit of work, more commits can be made to `feature_branch_name` (basically go back to step 3). These new commits will show up in the original pull request we made, there is **no need to create another new pull request**. Request that the reviewers look over the new changes to the pull request.
    - If the reviewers think everything looks ready to be merged, they can click on the big green _Merge pull request_ button.
        - Whether to merge, squash, or rebase the pull request basically boils down to preference, see [here](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/about-pull-request-merges) for more.
        - The pull request will only be able to be merged if there are no merge conflicts. If there are, go fix those first.


### Tips

- Don't work on multiple features in a single feature branch. Try and make feature branches as **focused on a single feature** as possible, this makes it simple to know what kind of work is happening on the feature branch and will make it easy to create small and contained pull requests later down the line.
- Keep pull requests **small and contained**: avoid pull requests that make large and many changes across the codebase (rule of thumb, specifically for this project IMO, ~500 lines changed max). This reduces the load on the pull request reviewers and makes it easy to see what changes the pull request intends on making.
- Don't be afraid to respectfully disagree or suggest changes; basically **be nice** when reviewing code :)
