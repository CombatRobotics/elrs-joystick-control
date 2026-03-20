// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

export const ModelIDMin = 0;
export const ModelIDMax = 63;
const ModelIDSuffix = "|model_id=";

export function normalizeModelID(value) {
    let parsed = parseInt(value, 10);
    if (Number.isNaN(parsed)) {
        return null;
    }
    if (parsed < ModelIDMin || parsed > ModelIDMax) {
        return null;
    }
    return parsed;
}

export function encodeStartLinkPortSpec(port, modelID) {
    if (!port || typeof port !== "string") {
        return "";
    }

    let normalized = normalizeModelID(modelID);
    if (normalized === null) {
        return `${port}`;
    }

    return `${port}${ModelIDSuffix}${normalized}`;
}

export function decodeStartLinkPortSpec(portSpec) {
    if (!portSpec || typeof portSpec !== "string") {
        return {port: "", modelID: 0};
    }

    let index = portSpec.lastIndexOf(ModelIDSuffix);
    if (index < 0) {
        return {port: portSpec, modelID: 0};
    }

    let port = portSpec.slice(0, index);
    let modelID = normalizeModelID(portSpec.slice(index + ModelIDSuffix.length));
    if (modelID === null) {
        return {port, modelID: 0};
    }

    return {port, modelID};
}
