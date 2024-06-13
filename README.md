# Website information

This website uses the jekyll theme [jekyll-nagymaros](https://github.com/piazzai/jekyll-nagymaros).

The website is built by using Github actions. The specific action is configured [here](.github/workflows/jekyll.yml)

# Build locally

> Following the instructions provided [here](https://github.com/BretFisher/jekyll-serve).

For development purposes, you can use the Docker image `bretfisher/jekyll` to build the page without requiring any installation of ruby, jekyll, etc. Simply run:
```bash
docker run -p 4000:4000 -v $(pwd):/site -it --entrypoint bash bretfisher/jekyll
```
Then run your commands interactively:
```bash
bundle install --retry 5 --jobs 20
bundle exec jekyll build
```
Then, your bind-mounted `_site` will be there on your host.

## Serve locally

Allow the Docker container to forward network ports to your computer by running:
```bash
docker run --rm -it --privileged --net=host --ipc=host -v $(pwd):/site -it --entrypoint bash bretfisher/jekyll
```
Then run your commands interactively:
```bash
bundle install --retry 5 --jobs 20
bundle exec jekyll serve
```
Finally, use your browser to visit `localhost:4000`.