// frontend/src/canvas/RobotRenderer.ts
/**
 * Robot rendering utilities.
 * Draws the robot based on URDF dimensions.
 */

export interface RobotShape {
    type: "box" | "cylinder" | "a300";
    length: number;
    width: number;
    wheel_base?: number;
    wheel_track?: number;
    wheel_radius?: number;
    wheel_width?: number;
}

// Default Husky A300 dimensions
export const DEFAULT_ROBOT: RobotShape = {
    type: "a300",
    length: 0.860,
    width: 0.378,
    wheel_base: 0.512,
    wheel_track: 0.562,
    wheel_radius: 0.1651,
    wheel_width: 0.1143,
};

/**
 * Draw robot at origin (0,0) facing +X direction.
 * Caller should translate and rotate first.
 */
export function drawRobot(
    ctx: CanvasRenderingContext2D,
    dims: RobotShape = DEFAULT_ROBOT,
    scale: number = 1
): void {
    ctx.save();
    ctx.scale(scale, scale);

    if (dims.type === "a300") {
        drawA300(ctx, dims);
    } else if (dims.type === "cylinder") {
        drawCylinder(ctx, dims);
    } else {
        drawBox(ctx, dims);
    }

    ctx.restore();
}

function drawA300(ctx: CanvasRenderingContext2D, dims: RobotShape): void {
    const chassisL = dims.length;
    const chassisW = dims.width;
    const wheelBase = dims.wheel_base || 0.512;
    const wheelTrack = dims.wheel_track || 0.562;
    const wheelRad = dims.wheel_radius || 0.1651;
    const wheelW = dims.wheel_width || 0.1143;
    const wheelDiam = wheelRad * 2;

    const colChassis = "#f59e0b";
    const colDark = "#333333";
    const colBumper = "#1f2937";

    // Suspension beams
    const beamW = 0.05;
    ctx.fillStyle = colDark;
    ctx.fillRect(-wheelBase / 2, wheelTrack / 2 - beamW / 2, wheelBase, beamW);
    ctx.fillRect(-wheelBase / 2, -wheelTrack / 2 - beamW / 2, wheelBase, beamW);

    // Wheels
    ctx.fillStyle = colDark;
    ctx.fillRect(wheelBase / 2 - wheelDiam / 2, wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
    ctx.fillRect(wheelBase / 2 - wheelDiam / 2, -wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
    ctx.fillRect(-wheelBase / 2 - wheelDiam / 2, wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
    ctx.fillRect(-wheelBase / 2 - wheelDiam / 2, -wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);

    // Chassis
    ctx.fillStyle = colChassis;
    ctx.fillRect(-chassisL / 2, -chassisW / 2, chassisL, chassisW);

    // Bumpers
    const bumperDepth = 0.05;
    const bumperWidth = chassisW + 0.02;
    ctx.fillStyle = colBumper;
    ctx.fillRect(chassisL / 2, -bumperWidth / 2, bumperDepth, bumperWidth);
    ctx.fillRect(-chassisL / 2 - bumperDepth, -bumperWidth / 2, bumperDepth, bumperWidth);

    // Direction indicator
    ctx.fillStyle = "rgba(0,0,0,0.2)";
    ctx.fillRect(chassisL * 0.1, -chassisW * 0.3, chassisL * 0.3, chassisW * 0.6);

    // Arrow
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 0.02;
    ctx.beginPath();
    ctx.moveTo(chassisL * 0.3, 0);
    ctx.lineTo(chassisL * 0.45, 0);
    ctx.stroke();

    ctx.fillStyle = "#ffffff";
    ctx.beginPath();
    ctx.moveTo(chassisL * 0.45, 0);
    ctx.lineTo(chassisL * 0.4, chassisW * 0.05);
    ctx.lineTo(chassisL * 0.4, -chassisW * 0.05);
    ctx.fill();
}

function drawBox(ctx: CanvasRenderingContext2D, dims: RobotShape): void {
    const lPx = dims.length;
    const wPx = dims.width;
    ctx.fillStyle = "#f59e0b";
    ctx.fillRect(-lPx / 2, -wPx / 2, lPx, wPx);

    // Direction indicator
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 0.02;
    ctx.beginPath();
    ctx.moveTo(lPx / 4, 0);
    ctx.lineTo(lPx / 2, 0);
    ctx.stroke();
}

function drawCylinder(ctx: CanvasRenderingContext2D, dims: RobotShape): void {
    const radius = dims.width / 2;
    ctx.fillStyle = "#f59e0b";
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.fill();

    // Direction indicator
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 0.02;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(radius, 0);
    ctx.stroke();
}

/**
 * Parse URDF to extract robot dimensions.
 */
export function parseRobotDescription(urdf: string): RobotShape | null {
    try {
        const parser = new DOMParser();
        const xml = parser.parseFromString(urdf, "text/xml");

        // Try Xacro properties for A300
        const props = xml.getElementsByTagName("xacro:property");
        const valMap: Record<string, number> = {};
        for (let i = 0; i < props.length; i++) {
            const name = props[i].getAttribute("name");
            const valStr = props[i].getAttribute("value");
            if (name && valStr) {
                const v = parseFloat(valStr);
                if (!isNaN(v)) valMap[name] = v;
            }
        }

        if (valMap["chassis_x_size"] && valMap["chassis_y_size"]) {
            return {
                type: "a300",
                length: valMap["chassis_x_size"],
                width: valMap["chassis_y_size"],
                wheel_base: valMap["wheel_base"] || 0.512,
                wheel_track: valMap["wheel_track"] || 0.562,
                wheel_radius: valMap["a300_outdoor_wheel_radius"] || 0.1651,
                wheel_width: valMap["a300_outdoor_wheel_width"] || 0.1143,
            };
        }

        // Heuristic: Check for A300 mesh paths
        if (urdf.includes("meshes/a300")) {
            return DEFAULT_ROBOT;
        }

        // Try standard URDF parsing
        const link = xml.querySelector("link[name='base_link']") || xml.querySelector("link[name='base_footprint']");
        if (link) {
            const geometry = link.querySelector("visual geometry");
            if (geometry) {
                const box = geometry.querySelector("box");
                const cylinder = geometry.querySelector("cylinder");

                if (box) {
                    const sizeStr = box.getAttribute("size");
                    if (sizeStr) {
                        const parts = sizeStr.trim().split(/\s+/).map(Number);
                        if (parts.length >= 2) {
                            return { length: parts[0], width: parts[1], type: "box" };
                        }
                    }
                } else if (cylinder) {
                    const radius = Number(cylinder.getAttribute("radius"));
                    if (!isNaN(radius)) {
                        return { length: radius * 2, width: radius * 2, type: "cylinder" };
                    }
                }
            }
        }

        return null;
    } catch {
        return null;
    }
}
