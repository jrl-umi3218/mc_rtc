Testing locally
==

Pre-requisites
--

You must have some dependencies installed on your system:

```bash
$ sudo apt install ruby-dev ruby-bundler libxml2-dev
```

Then install the required packages:

```bash
$ bundle install
```

Serving locally
--

Execute the following commands:

```bash
$ bundle exec jekyll serve --trace
```

Then point your browser to localhost:4000

The site will be rebuilt every time you make changes to the source files so you only need to refresh your browser to see the changes.

Doxygen integration
--

Doxygen integration is made in CI. If you want to browse it locally simply copy the installed documentatio, e.g.:

```bash
$ cp -r ${CMAKE_INSTALL_PREFIX}/share/doc/mc_rtc/doxygen-html .
```

HTML proofer
--

You can run the generated site through the HTML proofer tool to check for errors:

```bash
bundle exec htmlproofer ./_site --only-4xx --check-favicon --check-html --allow-hash-href --url-ignore "/gite.lirmm.fr/"
```

Adding a tutorial
==

1. Edit `_data/tutorials.yml` and insert your tutorial in the correct category
2. Create your page in `tutorials/{category-id}/{tutorial-id}.html` or `tutorials/{category-id}/{tutorial-id}.md`
3. The first lines of your page *must* be:
```yaml
---
layout: tutorials
---
```
4. Write your tutorial, for multi-language code snippets look at the existing tutorials
