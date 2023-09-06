import pickle
with open('data.json','rb') as f:
    data = pickle.load(f)

with open('data1.json', 'wb') as fp:
    pickle.dump(data, fp)

print(data)