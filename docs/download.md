# Download

## Manual Download

lebron can be downloaded as a [PROS](https://pros.cs.purdue.edu/) template from the [releases](https://github.com/lebron/lebron/releases) tab in the lebron github repository.

## Depot Download

If you don't want to re-download lebron every time a new release comes out, we've set up a depot to make the updating process easier.

You can use the following commands to add the depot to your `pros-cli` installation.

```bash
pros c add-depot lebron https://raw.githubusercontent.com/lebron/lebron/depot/stable.json # adds lebron's stable depot
pros c apply lebron # applies latest stable version of lebron
```

To update lebron, all you have to do is run the following command:

```bash
pros c update
```

### Beta Depot

```{warning}
Beta versions of lebron may not be fully tested or documented. Use at your own risk.
```

If you'd like to use a beta version of lebron you can add our beta depot like so:

```bash
pros c add-depot lebron https://raw.githubusercontent.com/lebron/lebron/depot/beta.json # adds lebron's beta depot
```

## Further steps

Once you've downloaded lebron we recommend you take a look at our [tutorials](./tutorials/1_getting_started.md).
