# Mehville RC 6744 Reefscape Development 2025
Updated 02/04/25

This project will develop software to run the Mehlville RC Reefscape robot. The Robot will feature swerve drive and an elevator with a manipulator to take-in and deposit  coral elements. REVrobotics/MAXSwerve-Java-Template provided the starting place for this effort.
### Development approach
The team will code and test all new features and fixes on specific feature branches. The several feature branches will be integrated and tested as a system on an integrate branch.
### GitHub Best Practices 
#### Terms:
- Codebase: all of our code for a robot
- Git: Source Control Manager, used for collaboration/tracking changes, where many terms in this list come from
- Repo: Short for Repository, where codebases is stored
- GitHub: Hosts our Repos, utilizes Git
- Commits: Blocks of Changes to our code
- Branch: Part of a repo that deviates from the master branch, think like tree branches
  - main: This is where production code goes. Never edit main directly
  - development: This is where code is tested before going into main. Do not edit this directly, instead please make a branch.
- Merge: To pull commits from one branch into another
- PR: Short for Pull Request, used to request a merge between branches
#### Branches:
- Here is some terminology we may use in branches
  - feat = Feature
  - int = Integration
  - bug = Bug Fix
- When making a new branch, name it appropriately
  - Not this:   AutonomousButReal (This was a real branch on last year's repo)
  - This:   feat-pigeon
#### Commits:
- Please make sure to commit changes to the right branch
  - NOT THIS:  Added Pigeon Object — to feat-swerve
  - THIS:  Added Gyro Reset Method — to feat-gyro
- Make sure commits are small and change as little as possible. This is so that we can track down bugs to a commit with one line, rather than a commit with many lines across many different files.
  - NOT THIS:  Commit changing the team number, adding a gyro method, and changing constants
  - THIS: Commit changing the team number, Commit adding a gyro method, and Commit changing constants
- Along with keeping commits themselves short, you should also keep your commit message short, and place any additional information in the commit description
  - **NEVER** THIS:  Commit: Commit (Never title Commits "Commit")
  - NOT THIS:  Commit: Changed Gyro Rotation Value in Constants from 1.0 to 1.25 (too much information in title)
  - THIS:  Commit: Changed Gyro Constant  
    Commit Description: Gyro Rotation: 1 --> 1.25
#### Pushing/Pulling:
- Code should be pulled from GitHub at the beginning of the day to avoid working with older versions of the code base
- Code should be pushed at the end of the day so that absent members can review commits themselves.
- If two or more people are working within the same branch (ideally this wouldn't happen) please meet up and do the following to avoid conflicts next meeting
  - Person 1 Pushes their commits
  - All other persons pull Person 1’s commits and resolve merge conflicts
  - Person 2 Pushes their commits
  - All other persons pull person 2’s commits and resolve merge conflicts
  - (Repeat with all people on the branch)
  - Person N-1 Pushes their commits
  - All other persons pull person N-1’s commits and resolve merge conflicts
  - Person N Pushes their commits
  - All persons can now leave
#### Issues:
- When making issues, Make sure the title is consice but informative. Put more information in the PR Description
- Make sure to link a branch to your Issue so that we can see the progress 
- Make sub-issues if needed
#### Merging/PRs:
- Make a PR Drafts with an issue attached
- Please test your code before marking your PR as ready
- Keep your title concise and on topic with extra info in the description
- There are protections on main so that people cannot add commits by themselves, the commits must be checked by two different members
