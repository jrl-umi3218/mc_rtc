#!/usr/bin/env sh

readonly doc_dir=`cd $(dirname $0); pwd`
mkdir -p $doc_dir/assets/css
mkdir -p $doc_dir/assets/js
cd $doc_dir/assets/css
wget -O bootstrap.min.css https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/css/bootstrap.min.css
wget -O octicons.css https://cdnjs.cloudflare.com/ajax/libs/octicons/8.0.0/build.css
cd $doc_dir/assets/js
wget -O jquery.min.js https://code.jquery.com/jquery-3.2.1.slim.min.js
wget -O popper.min.js https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.12.9/umd/popper.min.js
wget -O bootstrap.min.js https://maxcdn.bootstrapcdn.com/bootstrap/4.0.0/js/bootstrap.min.js
