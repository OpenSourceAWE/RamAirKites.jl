# Contributing

Contributions to this project are welcome. In particular we are looking for test cases and flight test data for
model validation.

## Plotting

For 2D plots, please use the MakieControlPlots package. If this is not sufficient, try to use GLMakie or CairoMakie directly.

## Modifying existing models

When modifying an existing model, submit your pull request to the **dev** branch.
Before creating the PR, make sure to rebase the **dev** branch onto the latest **main**.

The dev branch can then be used for extensive testing, updating of the examples (if needed), and tuning of the
controller parameters.

When all works well, create a pull request for the main branch.

- The main branch contains stable (perhaps outdated) models. All examples and controllers work well with these models
- The dev branch contains new models, not yet tested with the examples, and not yet with tuned controller settings