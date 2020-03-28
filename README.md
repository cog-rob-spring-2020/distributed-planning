# Distributed Planning

16.412 final project

## Development

* Python 3.8

All Python dependencies are in `requirements.txt`. We recommend using a [virtual environment](https://docs.python.org/3/library/venv.html) (venv) to manage dependencies.

If you're on Linux/Unix, here's a quick guide. If you're on Windows, see the Python [documentation](https://docs.python.org/3/library/venv.html).
1. `python -m venv .venv` (assuming `python` is version 3.8. You may need to adjust depending on how Python is installed on your system)
2. `. .venv/bin/activate`
3. `pip install -r requirements.txt`

### Testing

With your venv activated:

```sh
nose2
```

### Installing New Packages

Let's keep `requirements.txt` up to date with all required packages. With your venv activated:

```sh
pip install foo && pip freeze > requirements.txt
```
