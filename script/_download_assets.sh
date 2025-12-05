cd assets
python _download.py

# embodiments
unzip embodiments.zip
rm -rf embodiments.zip

cd ..
echo "Configuring Path ..."
python ./script/update_embodiment_config_path.py