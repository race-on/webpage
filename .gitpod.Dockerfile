FROM alpine:latest

ENV MKDOCS_VERSION=1.0.4

RUN \
    apk add --update \
        ca-certificates \
        bash \
        git \
        openssh \
        python3 \
        python3-dev && \
    pip3 install --upgrade pip && \
    pip install mkdocs==${MKDOCS_VERSION} && \
    pip install mkdocs-material mkdocs-git-revision-date-localized-plugin mkdocs-awesome-pages-plugin pyembed-markdown && \
    rm -rf /tmp/* /var/tmp/* /var/cache/apk/* /var/cache/distfiles/*
