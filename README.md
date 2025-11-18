# ROS 2 Connected Course - MkDocs Site

## Local preview
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
mkdocs serve
# open http://127.0.0.1:8000/
```

## Deploy to GitHub Pages
1. Push this repo to GitHub (default branch: `main`).
2. GitHub Actions will build and publish automatically using `.github/workflows/gh-pages.yml`.
3. In repo settings → Pages, set **Source: GitHub Actions**.

## Structure
- `mkdocs.yml` - site config (theme: Material)
- `docs/` - markdown content (index + tutorials + cheatsheet)
- `.github/workflows/gh-pages.yml` - Pages deploy workflow
