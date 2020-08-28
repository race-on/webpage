# webpage
Race On webpage source. Access the website here https://race-on.github.io/webpage/

# Change Page Content

To change the content of the website pages you would need to install mkdoks and few plugins used by the webpage. The easiest way to do that is to use conda:

```bash
conda create -n mkdocs python
conda activate mkdocs
pip install mkdocs-material mkdocs-git-revision-date-localized-plugin mkdocs-awesome-pages-plugin pyembed-markdown
```

To locally serve the website run ```mkdocs serve``` from the root of the repository.
