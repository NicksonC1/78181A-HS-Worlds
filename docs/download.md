c# Download

## Manual Download

genesis can be downloaded as a [PROS](https://pros.cs.purdue.edu/) template from the [releases](https://github.com/genesis/genesis/releases) tab in the genesis github repository.

## Depot Download

If you don't want to re-download genesis every time a new release comes out, we've set up a depot to make the updating process easier.

You can use the following commands to add the depot to your `pros-cli` installation.

```bash
pros c add-depot genesis https://raw.githubusercontent.com/genesis/genesis/depot/stable.json # adds genesis's stable depot
pros c apply genesis # applies latest stable version of genesis
```

To update genesis, all you have to do is run the following command:

```bash
pros c update
```

### Beta Depot

```{warning}
Beta versions of genesis may not be fully tested or documented. Use at your own risk.
```

If you'd like to use a beta version of genesis you can add our beta depot like so:

```bash
pros c add-depot genesis https://raw.githubusercontent.com/genesis/genesis/depot/beta.json # adds genesis's beta depot
```

## Further steps

Once you've downloaded genesis we recommend you take a look at our [tutorials](./tutorials/1_getting_started.md).
