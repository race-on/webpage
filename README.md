[![Gitpod Ready-to-Code](https://img.shields.io/badge/Gitpod-Ready--to--Code-blue?logo=gitpod)](https://gitpod.io/#https://github.com/race-on/webpage) 

When using Gitpod for the first time, before you can push your commits back to the GitHub repository you need to grant Gitpod write permissions. 
For that click on your avatar in the upper right corner and select "Open Access Control", check "write public repos", and click update. After confirming on the
Github page you will be able to push your commits to the GitHub repository.

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
