echo "#ifndef GIT_HASH"
echo -n "#define GIT_HASH \""

git rev-parse --short HEAD | tr -d "\n"
if ! git diff --quiet; then
  echo "-dirty\""
else
  echo "-clean\""
fi

echo "#endif"
