cp settings.json ~/settings.backup.json
git fetch
git reset --hard HEAD
git merge '@{u}'
npm i
cp ~/settings.backup.json settings.json
