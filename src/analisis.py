import json

with open('results.json') as json_file:
    results = json.load(json_file)

#[{file:"", optimo: 0, results: [{algorithm: "", planification: [], maxRP: 0}]}]


# Obtener todos los nombre de algoritmos
algorithms = []
for result in results:
    for r in result["results"]:
        if r["algorithm"] not in algorithms:
            algorithms.append(r["algorithm"])


#Contar por cada algoritmo, las veces que son iguales o mayores al optimo
result_optimo = {}
result_suboptimo = {}
for algorithm in algorithms:
    for result in results:
        for r in result["results"]:
            if r["algorithm"] == algorithm:
                # print(result["file"])
                # print(r["maxRP"], result["optimo"])
                if r["maxRP"] == result["optimo"]:
                    if algorithm not in result_optimo:
                        result_optimo[algorithm] = 1
                    else:
                        result_optimo[algorithm] += 1
                elif r["maxRP"] > result["optimo"]:
                    if algorithm not in result_suboptimo:
                        result_suboptimo[algorithm] = 1
                    else:
                        result_suboptimo[algorithm] += 1
                break
print("Número de casos optimos")
print(result_optimo)
print("Número de casos suboptimos")
print(result_suboptimo)
print()
print()
#Ver casos donde los algoritmos donde es mayor a 1.5 veces el optimo
result_mayor = {}
for algorithm in algorithms:
    for result in results:
        for r in result["results"]:
            if r["algorithm"] == algorithm:
                if r["maxRP"] > result["optimo"] and r["maxRP"] < 1.5 * result["optimo"]:
                    if algorithm not in result_mayor:
                        result_mayor[algorithm] = 1
                    else:
                        result_mayor[algorithm] += 1
                break
print("Número de casos donde es mayor a 1.5 veces el optimo")
print(result_mayor)
print()
print()

#comparar tiempos de ejecución promedio
result_time = {}
for algorithm in algorithms:
    for result in results:
        for r in result["results"]:
            if r["algorithm"] == algorithm:
                if algorithm not in result_time:
                    result_time[algorithm] = r["time"]
                else:
                    result_time[algorithm] += r["time"]
                break
for algorithm in algorithms:
    result_time[algorithm] = int(result_time[algorithm] / len(results))
print("Tiempo de ejecución promedio")
print(result_time)
print()
print()


#Ver casos donde alg[i] es mejor que el optimo
# result_solver = {}
# filenames = {}
# for algorithm in algorithms:
#     for result in results:
#         for r in result["results"]:
#             if r["algorithm"] == algorithm:
#                 if r["maxRP"] < result["optimo"]: # - 3:
#                     if algorithm not in result_solver:
#                         result_solver[algorithm] = 1
#                         filenames[algorithm] = [result["file"]]
#                     else:
#                         result_solver[algorithm] += 1
#                         filenames[algorithm].append(result["file"])
        
#                 break

# print("Número de casos donde alg[i] es mejor que el optimo")
# for algorithm in algorithms:
#     if algorithm not in result_solver:
#         continue
#     print(algorithm)
#     print(result_solver[algorithm])
#     print(filenames[algorithm])
# print()