
pip install ortools=='9.9.3963'

for file in $(ls instances/*.txt | sort -V); do
    echo "Running $file"
    output_file="outputs/sol_$(basename $file .txt).txt"
    python3 relaxed_sabanci.py --file-path "$file" --num-vehicles 75 --time-limit 60 > "$output_file"
    echo "Done"
done
