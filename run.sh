
pip install ortools=='9.9.3963'

for file in $(ls instances/*.txt | sort -V); do
    echo "Running $file"
    output_file="outputs/$(basename $file .txt).txt"
    python3 sabanci.py --file-path "$file" --num-vehicles 75 --time-limit 300 > "$output_file"
    echo "Done"
done
