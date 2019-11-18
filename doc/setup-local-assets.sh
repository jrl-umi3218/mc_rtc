#!/usr/bin/env sh

readonly doc_dir=`cd $(dirname $0); pwd`
mkdir -p $doc_dir/assets/css
mkdir -p $doc_dir/assets/js
cd $doc_dir/assets/css
wget -O bootstrap.min.css https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/css/bootstrap.min.css
wget -O octicons.css https://cdnjs.cloudflare.com/ajax/libs/octicons/8.0.0/build.css
cd $doc_dir/assets/js
wget -O jquery.min.js    https://code.jquery.com/jquery-3.3.1.slim.min.js
wget -O popper.min.js    https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.14.7/umd/popper.min.js
wget -O bootstrap.min.js https://stackpath.bootstrapcdn.com/bootstrap/4.3.1/js/bootstrap.min.js
wget -O clipboard.min.js https://cdnjs.cloudflare.com/ajax/libs/clipboard.js/2.0.4/clipboard.min.js
