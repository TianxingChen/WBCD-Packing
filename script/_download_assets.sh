cd assets
python _download.py

# background_texture
unzip background_texture.zip
rm -rf background_texture.zip

# embodiments
unzip embodiments.zip
rm -rf embodiments.zip

cd ..
echo "Configuring Path ..."
python ./script/update_embodiment_config_path.py