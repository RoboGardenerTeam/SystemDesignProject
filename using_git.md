# Our Git Workflow

Our git workflow is basically GitHub flow, as [described here](https://guides.github.com/introduction/flow/). I highly recommend taking a look at the link, it takes only 5 minutes to read.

Essentially the workflow is as follows:

1. You are on the `main` branch, and would like to work on a feature.
2. Instead of working directly on `main`, you create a **feature branch** to do all your work on.
    - Create and checkout a **new** feature branch (to simply switch to a branch that already exists, omit the `-b` flag).
        - `$ git checkout -b feature_branch_name`
    - At this point our feature branch is only a local branch (exists only our local machine). We wish to have this feature branch live on the shared repository as well so that others can work on it too.
        - `$ git push -u origin feature_branch_name`
            - The `-u` flag is to make the local feature branch a tracking branch, read more [here](https://git-scm.com/book/en/v2/Git-Branching-Remote-Branches). Don't forget to use this flag!
        - This pushes our local feature branch to the shared repository.
        - At this point the feature branch is a **shared** branch (
3. Do all your development work on the `feature_branch_name` branch.
    - This is your standard `git add`, `git commit`, and `git push`.
4. You feel like your work on the feature branch is ready to be merged into `main`. To do this, we open a **pull request**.
    - On the GitHub repo webpage, click the pull requests tab.
    - Click the big green button that says _New pull request_.
