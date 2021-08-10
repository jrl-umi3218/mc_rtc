#!/usr/bin/env sh

BOOTSTRAP_CSS="https://stackpath.bootstrapcdn.com/bootstrap/4.5.2/css/bootstrap.min.css"
JQUERY="https://code.jquery.com/jquery-3.5.1.slim.min.js"
POPPER="https://cdn.jsdelivr.net/npm/popper.js@1.16.1/dist/umd/popper.min.js"
BOOTSTRAP_JS="https://stackpath.bootstrapcdn.com/bootstrap/4.5.2/js/bootstrap.min.js"

readonly doc_dir=`cd $(dirname $0); pwd`
mkdir -p $doc_dir/assets/css
mkdir -p $doc_dir/assets/js
cd $doc_dir/assets/css
wget -O bootstrap.min.css $BOOTSTRAP_CSS
cd $doc_dir/assets/js
wget -O jquery.min.js    $JQUERY
wget -O popper.min.js    $POPPER
wget -O bootstrap.min.js $BOOTSTRAP_JS
wget -O clipboard.min.js https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.4/clipboard.min.js
