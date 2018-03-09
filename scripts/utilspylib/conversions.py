
def stringToMap(str, sep1=';', sep2='=', type1=int, type2=float):
    chunks = str.split(sep1)
    umap = {}
    for chunk in chunks:
        k, v = chunk.split(sep2)
        umap[type1(k)] = type2(v)
    return umap
