#!/bin/bash

if [ "$#" -lt 2 ]; then
	echo 'Intended usage'
    echo '    run_test TEST_NAME TEST_DIR [NPARALLEL]'
    echo ''
    echo '$BOOST_DIR and $COVERAGE_DIR must be defined.'
    exit
fi

TEST_NAME=$1
TEST_DIR=$2

TEST_NPARALLEL=2
if [ "$#" -ge 3 ]; then
	TEST_NPARALLEL=$3
fi

cd $BOOST_DIR

./b2 -j$TEST_NPARALLEL toolset=gcc cxxflags="--coverage" linkflags="--coverage" libs/geometry/$TEST_DIR

EXIT_STATUS=$?

cd ..

mkdir $COVERAGE_DIR/$TEST_NAME

files=$(find $BOOST_DIR/bin.v2/ -name "*.gcda")
for file in $files; do
  filen=${file##*/}
  filen=${filen%.*}
  dirn=${file%/*}
  dstfilen=$filen
  while [ -f $COVERAGE_DIR/$dstfilen.gcda ]; do
    dstfilen=$filen.$RANDOM
  done
  mv $dirn/$filen.gcda $COVERAGE_DIR/$TEST_NAME/$dstfilen.gcda
  mv $dirn/$filen.gcno $COVERAGE_DIR/$TEST_NAME/$dstfilen.gcno
done

cd $BOOST_DIR

rm -rf bin.v2/libs/geometry/$TEST_DIR

find "libs/geometry/$TEST_DIR" -type f -name "Jamfile" | while read -r file; do
    # Replace "run" with "compile" using sed
    sed -i 's/\brun\b/compile/g' "$file"
    sed -i 's/: : :/:/g' "$file"
done

./b2 -j$TEST_NPARALLEL -a toolset=gcc cxxflags="--std=c++17" libs/geometry/$TEST_DIR

exit $EXIT_STATUS
