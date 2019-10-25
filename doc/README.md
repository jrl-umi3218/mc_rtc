Testing locally
==

Pre-requisites
--

You must have gem and bundle installed on your system:

```bash
sudo apt install ruby-bundler
```

Then install the required packages:

```bash
bundle install
```

Serving locally
--

Execute the following commands:

```bash
bundle exec jekyll serve --trace
```

Then point your browser to localhost:4000

The site will be rebuilt every time you make changes to the source files so you only need to refresh your browser to see the changes.
