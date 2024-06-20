using DataFrames, DelimitedFiles

latex_content = """
\\begin{tabular}{|c|c|}
\\hline
A & B \\
\\hline
1 & 2 \\
3 & 4 \\
\\hline
\\end{tabular}
"""

# Extract lines of interest
lines = split(latex_content, '\n')
lines = filter(l -> !startswith(strip(l), "\\hline") && !startswith(strip(l), "\\begin") && !startswith(strip(l), "\\end"), lines)[1:end-1]

# Split by & and convert to DataFrame
data = [split(strip(l, ['\\']), "&") for l in lines]
matrix_data = DelimitedFiles.readdlm(IOBuffer(join(data, '&')), '&')
# Convert matrix data to appropriate format
clean_data = [eval(Meta.parse(val)) for val in matrix_data]
converted_data = [string.(v) for v in data] # Convert to Matrix{Vector{String}}
# Convert to DataFrame
df = DataFrame(permutedims(hcat(converted_data...)), [:col1, :col2])

df = DataFrame()


true_content = """
\\begin{table*}[]
\\caption{HERDS, PET, and Kresling configuration parameters for each of the space vehicles being considered made from carbon fiber}
\\centering
\\begin{tabular}{|c|ccc|ccc|ccc|ccc|ccc|}
\\hline
& \\multicolumn{3}{c|}{Falcon} & \\multicolumn{3}{c|}{Starship} & \\multicolumn{3}{c|}{Starship Ext.} & \\multicolumn{3}{c|}{SLS B1} & \\multicolumn{3}{c|}{SLS B2} \\\\
& HERDS & PET & Kresling & HERDS & PET & Kresling & HERDS & PET & Kresling & HERDS & PET & Kresling & HERDS & PET & Kresling \\\\ \\hline
l1 & 0.187 & - & - & 0.24 & - & - & 0.243 & - & - & 0.462 & - & - & 0.463 & - & - \\\\
l2 & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
l3 & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
t & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
α & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
β & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
n & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\ \\hline
r & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
ϕ & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
h & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
m & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\ \\hline
D & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
H₀   & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
H₁ & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
Mass & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\\\
% ... Continue in this pattern for all the other rows ...
\\hline
\\end{tabular}
\\label{tb:config-params}
\\end{table*}
"""
# Extract lines of interest
lines = split(true_content, '\n')
lines = filter(l -> !startswith(strip(l), "\\hline") && !startswith(strip(l), "\\begin") && !startswith(strip(l), "\\end"), lines)[1:end-1]

# Split by & and convert to DataFrame
data = [split(strip(l, ['\\']), "&") for l in lines]
matrix_data = DelimitedFiles.readdlm(IOBuffer(join(data, '&')), '&')
# Convert matrix data to appropriate format
clean_data = [eval(Meta.parse(val)) for val in matrix_data]
converted_data = [string.(v) for v in data] # Convert to Matrix{Vector{String}}
# Convert to DataFrame
df = DataFrame(permutedims(hcat(converted_data...)), [:col1, :col2])