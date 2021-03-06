# Student Assignment

16.412 final project - student assignment

---

## File Structure

```
/
|-- student/ - student-facing notebook and pset code, including tutorial
|-- solutions/  - instructor-facing algo solutions, some example notebooks for testing
|-- tests/      - instructor-facing unit tests
```

## Development

* Python 3.8

For sanity's sake, let's use 4 spaces. If you use VS Code, this repo includes settings that will automatically set tabs to 4 spaces.

All Python dependencies are in `requirements.txt`. We recommend using a [virtual environment](https://docs.python.org/3/library/venv.html) (venv) to manage dependencies.

If you're on Linux/Unix, here's a quick guide. If you're on Windows, see the Python [documentation](https://docs.python.org/3/library/venv.html).
1. `python -m venv .venv` (assuming `python` is version 3.8. You may need to adjust depending on how Python is installed on your system)
2. `. .venv/bin/activate`
3. `pip install -r requirements.txt`

## Jupyter Notebook Usage

```sh
PYTHONPATH="$PWD" jupyter notebook
```

When it loads, make sure you're using the kernel from your venv.

### Testing

With your venv activated:

```sh
PYTHONPATH="$PWD" nose2 --pretty-assert
```

### Installing New Packages

Let's keep `requirements.txt` up to date with all required packages. With your venv activated:

```sh
# make sure you've installed everything first or the next step will blow away
# packages you haven't installed
pip install -r requirements.txt

# actually install the new package (`foo` in this case) and then update
# the requirements
pip install foo && pip freeze > requirements.txt
```
