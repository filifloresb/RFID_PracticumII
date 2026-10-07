"""Reasigna materiales de un GLB exportado de Inventor a nombres que siteviewer
(MATLAB) reconoce: concrete / glass / metal. La geometría (chunk BIN) se copia
sin tocar; sólo se reescribe el JSON.

Uso:  python asignar_materiales_glb.py [entrada.glb] [salida.glb]
"""
import json
import struct
import sys

# Prefijo del nombre de la pieza raíz (ocurrencia en Inventor) -> material.
# Lo que no coincida usa DEFAULT.
REGLAS = [("ConcreteLab", "concrete"), ("GlassPanels", "glass")]
DEFAULT = "metal"

MATERIALES = {
    "concrete": {"pbrMetallicRoughness": {"baseColorFactor": [0.6, 0.6, 0.6, 1]}},
    "glass": {"pbrMetallicRoughness": {"baseColorFactor": [0.6, 0.8, 0.9, 0.35]}, "alphaMode": "BLEND"},
    "metal": {"pbrMetallicRoughness": {"baseColorFactor": [0.75, 0.75, 0.8, 1], "metallicFactor": 1}},
}


def leer_glb(ruta):
    data = open(ruta, "rb").read()
    magic, version, _ = struct.unpack_from("<4sII", data, 0)
    assert magic == b"glTF" and version == 2, "No es un GLB v2"
    largo_json, tipo = struct.unpack_from("<II", data, 12)
    assert tipo == 0x4E4F534A, "El primer chunk no es JSON"
    gltf = json.loads(data[20:20 + largo_json])
    resto = data[20 + largo_json:]  # chunk BIN (y cualquier otro) tal cual
    return gltf, resto


def escribir_glb(ruta, gltf, resto):
    js = json.dumps(gltf, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    js += b" " * (-len(js) % 4)
    total = 12 + 8 + len(js) + len(resto)
    with open(ruta, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A))
        f.write(js)
        f.write(resto)


def material_de(nombre):
    for prefijo, mat in REGLAS:
        if nombre.startswith(prefijo):
            return mat
    return DEFAULT


def reasignar(gltf):
    nombres = list(MATERIALES)
    idx = {m: i for i, m in enumerate(nombres)}
    nodos, mallas = gltf["nodes"], gltf["meshes"]
    asignada = {}  # malla -> material ya puesto (una malla puede reusarse en varias piezas)
    conteo = {m: 0 for m in nombres}

    def recorrer(i, mat):
        nodo = nodos[i]
        if "mesh" in nodo:
            m = nodo["mesh"]
            if asignada.get(m, mat) != mat:  # malla compartida con otra clase: duplicarla
                mallas.append(json.loads(json.dumps(mallas[m])))
                m = nodo["mesh"] = len(mallas) - 1
            asignada[m] = mat
            for prim in mallas[m]["primitives"]:
                prim["material"] = idx[mat]
                conteo[mat] += 1
        for hijo in nodo.get("children", []):
            recorrer(hijo, mat)

    for raiz in gltf["scenes"][gltf.get("scene", 0)]["nodes"]:
        nombre = nodos[raiz].get("name", "")
        mat = material_de(nombre)
        print(f"  {nombre:40s} -> {mat}")
        recorrer(raiz, mat)

    gltf["materials"] = [{"name": m, **MATERIALES[m]} for m in nombres]
    gltf["scene"] = gltf.get("scene", 0)  # Inventor no lo exporta y MATLAB lo exige
    # Las texturas/extensiones de Inventor ya no se referencian
    for clave in ("textures", "images", "samplers", "extensionsUsed", "extensionsRequired"):
        gltf.pop(clave, None)
    return conteo


def main():
    entrada = sys.argv[1] if len(sys.argv) > 1 else "LabRobotica_ASM.glb"
    salida = sys.argv[2] if len(sys.argv) > 2 else "LabRobotica_ASM_materiales.glb"
    gltf, resto = leer_glb(entrada)
    conteo = reasignar(gltf)
    escribir_glb(salida, gltf, resto)

    # Verificación: releer y confirmar que toda primitiva quedó con un material válido
    g2, _ = leer_glb(salida)
    n = len(g2["materials"])
    assert all(0 <= p.get("material", -1) < n for m in g2["meshes"] for p in m["primitives"])
    print(f"\nPrimitivas por material: {conteo}\nEscrito: {salida}")


if __name__ == "__main__":
    main()
