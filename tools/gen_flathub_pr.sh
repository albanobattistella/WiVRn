#!/bin/bash

set -e

IN=$(realpath $(dirname $0)/../flatpak/io.github.wivrn.wivrn.yml.in)
OUT=$(realpath $1)

cd $(dirname $0)

GIT_COMMIT=$(git describe --exact-match --tags 2>/dev/null || git rev-parse HEAD)

GIT_DESC=$(git describe --tags)
MONADO_COMMIT=$(grep -zo "FetchContent_Declare(monado[^)]*)" ../CMakeLists.txt | sed -ze "s/^.*GIT_TAG *\([^ )]*\).*$/\1/" | tr -d '\0')
BOOSTPFR_URL=$(grep -zo "FetchContent_Declare(boostpfr[^)]*)" ../CMakeLists.txt | sed -ze "s/^.*URL *\([^ )]*\).*$/\1/" | tr -d '\0')
BOOSTPFR_SHA256=$(curl --silent --location $BOOSTPFR_URL | sha256sum | cut -f1 -d' ')

echo "Input:  $IN"
echo "Output: $OUT"
echo
echo "Git commit:       $GIT_COMMIT"
echo "Git tag:          $GIT_DESC"
echo "Monado commit:    $MONADO_COMMIT"
echo "Boost.PFR URL:    $BOOSTPFR_URL"
echo "Boost.PFR SHA256: $BOOSTPFR_SHA256"

cat $IN | sed \
    -e s/WIVRN_GIT_COMMIT/$GIT_COMMIT/     \
    -e s/WIVRN_GIT_DESC/$GIT_DESC/         \
    -e s,BOOSTPFR_URL,$BOOSTPFR_URL,       \
    -e s/BOOSTPFR_SHA256/$BOOSTPFR_SHA256/ \
    -e s/MONADO_COMMIT/$MONADO_COMMIT/     > $OUT
