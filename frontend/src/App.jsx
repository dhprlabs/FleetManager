import { useEffect, useRef, useState } from "react";
import warehouseMap from "./asset/logistics_warehouse.png";
import { useRos, rosToSvg, svgToRos, DEFAULT_AISLES, normalizeAisles } from "./useRos";

const baseViewBox = { x: 0, y: 0, w: 485, h: 484 };
const axisEnd = 600;
const now = 220;
const colors = {
    pine: "#2f6f5e",
    pineDim: "#e4efe9",
    amber: "#c97a2b",
    amberDim: "#f7ebdc",
    rust: "#c0453b",
    rustDim: "#f6e4e1",
    robot: "#3978b7",
    robotDim: "#e4edf7",
    offline: "#b7beb8",
};
const font = { fontFamily: "IBM Plex Sans, sans-serif" };
const displayFont = { fontFamily: "Space Grotesk, sans-serif" };

function statusColor(robot) {
    return robot.online ? robot.color : colors.offline;
}

// ── Single-lane aisle coordinate & occupancy helpers ─────────────────────────
function aisleToSvgRect(aisle) {
    const x1 = Math.min(aisle.x_min, aisle.x_max);
    const x2 = Math.max(aisle.x_min, aisle.x_max);
    const y1 = Math.min(aisle.y_min, aisle.y_max);
    const y2 = Math.max(aisle.y_min, aisle.y_max);
    const pTopLeft = rosToSvg(x1, y2);
    const pBottomRight = rosToSvg(x2, y1);
    return {
        x: pTopLeft.x,
        y: pTopLeft.y,
        width: Math.max(2, pBottomRight.x - pTopLeft.x),
        height: Math.max(2, pBottomRight.y - pTopLeft.y),
    };
}

function getAisleOccupancy(aisleId, trafficEvents = []) {
    for (let i = trafficEvents.length - 1; i >= 0; i--) {
        const ev = trafficEvents[i];
        if (ev.segment_id === aisleId) {
            if (ev.event === 'reservation_granted' || ev.event === 'reservation_active') {
                return { occupied: true, holder: ev.robot_id, event: ev.event };
            }
            if (ev.event === 'reservation_released' || ev.event === 'reservation_expired' || ev.event === 'reservation_denied') {
                break;
            }
        }
    }
    return { occupied: false, holder: null, event: null };
}

function RobotStatus({ robot }) {
    const batteryColor =
        robot.battery > 50
            ? colors.pine
            : robot.battery > 20
                ? colors.amber
                : colors.rust;
    return (
        <div className="mb-2 flex items-center gap-[11px] rounded-[9px] border border-[#eceee9] bg-[#f7f8f6] p-2.5">
            <div
                className="relative flex h-[34px] w-[34px] flex-none items-center justify-center rounded-[9px] border border-[#e3e6e1] bg-white text-xs font-semibold text-[#6b776f]"
                style={displayFont}
            >
                {robot.id.slice(-2)}
                <span
                    className="absolute -bottom-0.5 -right-0.5 h-[9px] w-[9px] rounded-full border-2 border-[#f7f8f6]"
                    style={{ backgroundColor: statusColor(robot) }}
                />
            </div>
            <div className="min-w-0 flex-1">
                <div className="text-[12.5px] font-semibold">{robot.id}</div>
                <div className="mt-0.5 truncate text-[11px] text-[#6b776f]">
                    {robot.online ? (robot.task && robot.task !== '—' ? robot.task : 'Idle') : 'Under maintenance'}
                </div>
            </div>
            <div className="flex-none text-right">
                <div className="flex items-center justify-end gap-[5px] text-[11.5px] font-medium">
                    <span className="h-[10px] w-5 rounded-sm border border-[#8e988f] p-[1.5px]">
                        <span
                            className="block h-full rounded-[1px]"
                            style={{
                                width: `${robot.battery}%`,
                                backgroundColor: batteryColor,
                            }}
                        />
                    </span>
                    {robot.battery}%
                </div>
                <div className="mt-0.5 text-[10px] text-[#8e988f]">
                    {robot.online
                        ? robot.task && robot.task !== '—'
                            ? robot.state === 'warn' ? 'Low battery' : 'Active'
                            : 'Idle'
                        : 'Offline'}
                </div>
            </div>
        </div>
    );
}


function RobotStatusPanel({ robots }) {
    return (
        <section className="flex w-full flex-none flex-col border-b border-[#c4cbc5] bg-white md:w-[260px] md:border-b-0 md:border-r md:border-[#c4cbc5] xl:w-[340px]">
            <div className="flex flex-none items-center justify-between px-[18px] pb-2.5 pt-3.5">
                <h2 className="m-0 text-[15px] font-semibold" style={displayFont}>
                    ROBOT <i>STATUS</i>
                </h2>
                <span className="text-xs text-[#8e988f]">{robots.length} units</span>
            </div>
            <div className="flex-1 overflow-y-auto px-3.5 pb-3.5">
                {robots.map((robot) => (
                    <RobotStatus key={robot.id} robot={robot} />
                ))}
            </div>
        </section>
    );
}

function MapPanel({
    robots,
    selectionMode,
    selectedPoints,
    tasks,
    stagedTasks = [],
    onPointSelect,
    dockInfo,
    aisles = [],
    selectedAisleId = null,
    onSelectAisle = () => { },
    aisleDrawingMode = false,
    aisleCorners = [],
    onAisleCornerSelect = () => { },
    onCancelAisleDraw = () => { },
    onOpenAisleManager = () => { },
    trafficEvents = [],
    onZoomChange = () => { },
}) {
    const svgRef = useRef(null);
    const instructionRef = useRef(null);
    const [viewBox, setViewBox] = useState(baseViewBox);
    const [drag, setDrag] = useState(null);
    const [mapPixels, setMapPixels] = useState(null);
    const [blockedMessage, setBlockedMessage] = useState("");
    const [instructionHovered, setInstructionHovered] = useState(false);
    const [hoverMapPoint, setHoverMapPoint] = useState(null);
    const zoom = Math.round((baseViewBox.w / viewBox.w) * 100);
    const currentTaskNum = stagedTasks.length + 1;

    useEffect(() => {
        function reportScale() {
            const rect = svgRef.current?.getBoundingClientRect();
            if (!rect || !rect.width || !rect.height) return;
            const pixelsPerMapUnit = Math.min(rect.width / viewBox.w, rect.height / viewBox.h);
            onZoomChange?.({
                zoom,
                pixelsPerMeter: pixelsPerMapUnit * 20,
            });
        }

        reportScale();
        const observer = new ResizeObserver(reportScale);
        if (svgRef.current) observer.observe(svgRef.current);
        return () => observer.disconnect();
    }, [onZoomChange, viewBox.h, viewBox.w, zoom]);

    function activeLabelPosition(point) {
        const overlapsExisting = [...tasks, ...stagedTasks].some((task) =>
            [task.start, task.end].some((endpoint) =>
                endpoint && Math.hypot(endpoint.x - point.x, endpoint.y - point.y) < 28,
            ),
        );
        return overlapsExisting
            ? { x: point.x + 18, y: point.y + 24 }
            : { x: point.x + 13, y: point.y - 10 };
    }

    useEffect(() => {
        const image = new Image();
        image.onload = () => {
            const canvas = document.createElement("canvas");
            canvas.width = image.naturalWidth;
            canvas.height = image.naturalHeight;
            const context = canvas.getContext("2d");
            context.drawImage(image, 0, 0);
            setMapPixels({
                data: context.getImageData(0, 0, canvas.width, canvas.height),
                width: canvas.width,
                height: canvas.height,
            });
        };
        image.src = warehouseMap;
    }, []);

    useEffect(() => {
        if (!selectionMode) setBlockedMessage("");
        if (!selectionMode) setInstructionHovered(false);
    }, [selectionMode]);

    useEffect(() => {
        if (!blockedMessage) return undefined;
        const timeout = window.setTimeout(() => setBlockedMessage(""), 3200);
        return () => window.clearTimeout(timeout);
    }, [blockedMessage]);
    function zoomBy(
        factor,
        cx = viewBox.x + viewBox.w / 2,
        cy = viewBox.y + viewBox.h / 2,
    ) {
        const width = Math.min(
            baseViewBox.w * 2.2,
            Math.max(baseViewBox.w * 0.25, viewBox.w * factor),
        );
        const height = width * (baseViewBox.h / baseViewBox.w);
        setViewBox({
            x: cx - ((cx - viewBox.x) / viewBox.w) * width,
            y: cy - ((cy - viewBox.y) / viewBox.h) * height,
            w: width,
            h: height,
        });
    }
    function handleWheel(event) {
        event.preventDefault();
        const point = svgRef.current.createSVGPoint();
        point.x = event.clientX;
        point.y = event.clientY;
        const location = point.matrixTransform(
            svgRef.current.getScreenCTM().inverse(),
        );
        zoomBy(event.deltaY > 0 ? 1.1 : 0.9, location.x, location.y);
    }
    function handlePointerMove(event) {
        const point = getMapPoint(event);
        setHoverMapPoint({ x: Math.round(point.x), y: Math.round(point.y) });
        const instructionBounds = instructionRef.current?.getBoundingClientRect();
        setInstructionHovered(Boolean(
            instructionBounds &&
            event.clientX >= instructionBounds.left &&
            event.clientX <= instructionBounds.right &&
            event.clientY >= instructionBounds.top &&
            event.clientY <= instructionBounds.bottom,
        ));
        if (!drag) return;
        const rect = svgRef.current.getBoundingClientRect();
        const dx = (event.clientX - drag.startX) * (viewBox.w / rect.width);
        const dy = (event.clientY - drag.startY) * (viewBox.h / rect.height);
        setViewBox({ ...viewBox, x: drag.viewBox.x - dx, y: drag.viewBox.y - dy });
    }
    function getMapPoint(event) {
        const point = svgRef.current.createSVGPoint();
        point.x = event.clientX;
        point.y = event.clientY;
        return point.matrixTransform(svgRef.current.getScreenCTM().inverse());
    }
    function handleMapClick(event) {
        if (drag) return;
        const point = getMapPoint(event);
        if (point.x < 0 || point.x > baseViewBox.w || point.y < 0 || point.y > baseViewBox.h) return;

        if (aisleDrawingMode) {
            onAisleCornerSelect({ x: Math.round(point.x), y: Math.round(point.y) });
            return;
        }

        if (!selectionMode) return;

        if (mapPixels) {
            const imageX = Math.min(mapPixels.width - 1, Math.max(0, Math.floor((point.x / baseViewBox.w) * mapPixels.width)));
            const imageY = Math.min(mapPixels.height - 1, Math.max(0, Math.floor((point.y / baseViewBox.h) * mapPixels.height)));
            const pixelIndex = (imageY * mapPixels.width + imageX) * 4;
            const red = mapPixels.data.data[pixelIndex];
            const green = mapPixels.data.data[pixelIndex + 1];
            // In logistics_warehouse.png: 0 = wall, 205 = unknown/outside, 254 = free floor
            const isBlocked = red < 128 || (red === 205 && green === 205);
            if (isBlocked) {
                setBlockedMessage(red < 128 ? "That point is on a wall. Choose an open floor area." : "That point is outside mapped warehouse area.");
                return;
            }
        }

        setBlockedMessage("");
        onPointSelect({ x: Math.round(point.x), y: Math.round(point.y) });
    }
    return (
        <section className="relative flex min-h-[300px] flex-1 flex-col overflow-hidden bg-[#CFCFCF]">
            <svg
                ref={svgRef}
                className={`block min-h-0 w-full flex-1 select-none bg-[#CFCFCF] ${aisleDrawingMode || selectionMode ? "cursor-crosshair" : drag ? "cursor-grabbing" : "cursor-grab"}`}
                style={{ userSelect: "none" }}
                viewBox={`${viewBox.x} ${viewBox.y} ${viewBox.w} ${viewBox.h}`}
                onWheel={handleWheel}
                onPointerDown={(event) => {
                    setDrag({ startX: event.clientX, startY: event.clientY, viewBox });
                    event.currentTarget.setPointerCapture(event.pointerId);
                }}
                onPointerMove={handlePointerMove}
                onPointerLeave={() => setInstructionHovered(false)}
                onPointerUp={() => setDrag(null)}
                onClick={handleMapClick}
            >
                {/* 1:1 Pixel-perfect logistics warehouse map (485 × 484 px = 24.25 × 24.2 m @ 0.05m/px) */}
                <image
                    href={warehouseMap}
                    x="0"
                    y="0"
                    width="485"
                    height="484"
                    preserveAspectRatio="xMidYMid meet"
                    opacity="1"
                />

                {/* Single-Lane Aisle Mutual Exclusion Zones */}
                <g id="single-lane-aisles-layer">
                    {aisles.map((aisle) => {
                        const rect = aisleToSvgRect(aisle);
                        const occ = getAisleOccupancy(aisle.segment_id || aisle.id, trafficEvents);
                        const isSelected = selectedAisleId === (aisle.segment_id || aisle.id);
                        const strokeColor = occ.occupied ? "#f59e0b" : isSelected ? "#3b82f6" : "#6366f1";
                        const fillColor = occ.occupied ? "#fef3c7" : isSelected ? "#e0e7ff" : "#eef2ff";
                        const fillOpacity = occ.occupied ? 0.38 : isSelected ? 0.32 : 0.16;

                        return (
                            <g
                                key={aisle.segment_id || aisle.id}
                                className="cursor-pointer"
                                onClick={(e) => {
                                    e.stopPropagation();
                                    onSelectAisle(aisle);
                                }}
                            >
                                {occ.occupied && (
                                    <rect
                                        x={rect.x - 2}
                                        y={rect.y - 2}
                                        width={rect.width + 4}
                                        height={rect.height + 4}
                                        rx="4"
                                        fill="none"
                                        stroke="#f59e0b"
                                        strokeWidth="1.2"
                                        strokeDasharray="4 2"
                                        className="animate-[pulse_2s_ease-in-out_infinite]"
                                    />
                                )}
                                <rect
                                    x={rect.x}
                                    y={rect.y}
                                    width={rect.width}
                                    height={rect.height}
                                    rx="3"
                                    fill={fillColor}
                                    fillOpacity={fillOpacity}
                                    stroke={strokeColor}
                                    strokeWidth={isSelected ? "2.2" : "1.2"}
                                    strokeDasharray={occ.occupied ? "none" : "3 3"}
                                />
                                {/* Clean status label badge */}
                                <g transform={`translate(${rect.x + 3}, ${rect.y + 3})`}>
                                    <rect
                                        x="0"
                                        y="0"
                                        width={Math.min(Math.max(46, (aisle.name || aisle.id).length * 5.2 + 26), Math.max(22, rect.width - 6))}
                                        height="12"
                                        rx="2"
                                        fill={occ.occupied ? "#b45309" : isSelected ? "#1d4ed8" : "#4338ca"}
                                        fillOpacity="0.88"
                                    />
                                    <text
                                        x="4"
                                        y="9"
                                        fill="#ffffff"
                                        fontFamily="IBM Plex Sans, sans-serif"
                                        fontSize="7.5"
                                        fontWeight="700"
                                    >
                                        {aisle.name || aisle.id} {occ.occupied ? `• ${occ.holder}` : "• 1-Way"}
                                    </text>
                                </g>
                            </g>
                        );
                    })}

                    {/* Interactive Drawing Preview */}
                    {aisleDrawingMode && aisleCorners.length === 1 && (
                        <g>
                            <circle
                                cx={aisleCorners[0].x}
                                cy={aisleCorners[0].y}
                                r="5"
                                fill="#6366f1"
                                stroke="white"
                                strokeWidth="2"
                            />
                            {hoverMapPoint && (
                                <>
                                    <rect
                                        x={Math.min(aisleCorners[0].x, hoverMapPoint.x)}
                                        y={Math.min(aisleCorners[0].y, hoverMapPoint.y)}
                                        width={Math.max(2, Math.abs(hoverMapPoint.x - aisleCorners[0].x))}
                                        height={Math.max(2, Math.abs(hoverMapPoint.y - aisleCorners[0].y))}
                                        rx="3"
                                        fill="#6366f1"
                                        fillOpacity="0.22"
                                        stroke="#6366f1"
                                        strokeWidth="1.8"
                                        strokeDasharray="4 3"
                                    />
                                    {(() => {
                                        const r1 = svgToRos(aisleCorners[0].x, aisleCorners[0].y);
                                        const r2 = svgToRos(hoverMapPoint.x, hoverMapPoint.y);
                                        const dx = Math.abs(r2.x - r1.x).toFixed(2);
                                        const dy = Math.abs(r2.y - r1.y).toFixed(2);
                                        const midX = (aisleCorners[0].x + hoverMapPoint.x) / 2;
                                        const minY = Math.min(aisleCorners[0].y, hoverMapPoint.y);
                                        return (
                                            <text
                                                x={midX}
                                                y={Math.max(12, minY - 6)}
                                                fill="#4338ca"
                                                fontSize="9"
                                                fontWeight="700"
                                                textAnchor="middle"
                                                style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                                            >
                                                {`Corridor bounds (${dx}m × ${dy}m)`}
                                            </text>
                                        );
                                    })()}
                                </>
                            )}
                        </g>
                    )}
                </g>

                {/* Task routes & markers */}
                {tasks.map((task) => task.start && task.end && (
                    <g key={`${task.name}-${task.number}`}>
                        {/* Connecting trajectory between pickup and dropoff */}
                        <line
                            x1={task.start.x}
                            y1={task.start.y}
                            x2={task.end.x}
                            y2={task.end.y}
                            stroke={task.robot ? colors.pine : colors.amber}
                            strokeWidth="1.8"
                            strokeDasharray="4 3"
                            strokeOpacity="0.7"
                        />
                        {/* Pickup marker (Start) */}
                        <circle
                            cx={task.start.x}
                            cy={task.start.y}
                            r="6"
                            fill={colors.rust}
                            stroke="white"
                            strokeWidth="1.5"
                        />
                        <text
                            x={task.start.x}
                            y={task.start.y - 8}
                            fill={colors.rust}
                            fontSize="8"
                            fontWeight="700"
                            textAnchor="middle"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.9))" }}
                        >
                            {`P${task.number || task.name}`}
                        </text>
                        {/* Dropoff marker (End) */}
                        <circle
                            cx={task.end.x}
                            cy={task.end.y}
                            r="6"
                            fill={colors.pine}
                            stroke="white"
                            strokeWidth="1.5"
                        />
                        <text
                            x={task.end.x}
                            y={task.end.y - 8}
                            fill={colors.pine}
                            fontSize="8"
                            fontWeight="700"
                            textAnchor="middle"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.9))" }}
                        >
                            {`D${task.number || task.name}`}
                        </text>
                    </g>
                ))}

                {/* Staged Tasks (Ready to Broadcast) */}
                {stagedTasks.map((task) => task.start && task.end && (
                    <g key={`staged-${task.id || task.name}`}>
                        <line
                            x1={task.start.x}
                            y1={task.start.y}
                            x2={task.end.x}
                            y2={task.end.y}
                            stroke="#e67e22"
                            strokeWidth="2"
                            strokeDasharray="4 3"
                            strokeOpacity="0.85"
                        />
                        {/* Staged Pickup */}
                        <circle cx={task.start.x} cy={task.start.y} r="6" fill="#e67e22" stroke="white" strokeWidth="1.5" />
                        <text
                            x={task.start.x}
                            y={task.start.y - 8}
                            fill="#e67e22"
                            fontSize="8"
                            fontWeight="800"
                            textAnchor="middle"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                        >
                            {`P${task.number || task.name}`}
                        </text>
                        {/* Staged Dropoff */}
                        <circle cx={task.end.x} cy={task.end.y} r="6" fill="#27ae60" stroke="white" strokeWidth="1.5" />
                        <text
                            x={task.end.x}
                            y={task.end.y - 8}
                            fill="#27ae60"
                            fontSize="8"
                            fontWeight="800"
                            textAnchor="middle"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                        >
                            {`D${task.number || task.name}`}
                        </text>
                    </g>
                ))}

                {/* Point selection preview when creating new tasks */}
                {selectionMode && selectedPoints.map((point, index) => (
                    <g key={`${point.x}-${point.y}`}>
                        <circle
                            cx={point.x}
                            cy={point.y}
                            r="7"
                            fill={index === 0 ? colors.rust : colors.pine}
                            stroke="white"
                            strokeWidth="2"
                        />
                        <text
                            x={activeLabelPosition(point).x}
                            y={activeLabelPosition(point).y}
                            fill={index === 0 ? colors.rust : colors.pine}
                            fontSize="10"
                            fontWeight="700"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.9))" }}
                        >
                            {index === 0 ? `Pickup (T${currentTaskNum})` : `Dropoff (T${currentTaskNum})`}
                        </text>
                    </g>
                ))}

                {/* Task Broadcaster Station & RF Coverage (Light & Compact) */}
                {dockInfo && (
                    <g id="task-broadcaster-layer">
                        {/* Subtle, Lightweight RF Coverage Zone */}
                        <circle
                            cx={dockInfo.x}
                            cy={dockInfo.y}
                            r={dockInfo.radiusPx}
                            fill="#f59e0b"
                            fillOpacity="0.035"
                            stroke="#f59e0b"
                            strokeDasharray="4 4"
                            strokeWidth="1"
                            strokeOpacity="0.45"
                        />

                        {/* Compact Dock Station Base Pad (12x12 px, matching robot scale) */}
                        <rect
                            x={dockInfo.x - 6}
                            y={dockInfo.y - 6}
                            width="12"
                            height="12"
                            rx="3"
                            fill="#ffffff"
                            stroke="#f59e0b"
                            strokeWidth="1.5"
                            style={{ filter: "drop-shadow(0px 1px 3px rgba(0,0,0,0.15))" }}
                        />
                        {/* Broadcaster antenna core dot */}
                        <circle
                            cx={dockInfo.x}
                            cy={dockInfo.y}
                            r="2.5"
                            fill="#f59e0b"
                        />
                        <circle
                            cx={dockInfo.x}
                            cy={dockInfo.y}
                            r="0.8"
                            fill="#ffffff"
                        />

                        {/* Clean, lightweight side labels matching robot label styling */}
                        <text
                            x={dockInfo.x + 9}
                            y={dockInfo.y - 2}
                            fill="#92400e"
                            fontFamily="IBM Plex Sans, sans-serif"
                            fontSize="8"
                            fontWeight="700"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                        >
                            Broadcaster
                        </text>
                        <text
                            x={dockInfo.x + 9}
                            y={dockInfo.y + 7}
                            fill="#78350f"
                            fontFamily="IBM Plex Sans, sans-serif"
                            fontSize="7"
                            fontWeight="500"
                            style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                        >
                            6m RF range
                        </text>
                    </g>
                )}

                {/* Fleet Robots */}
                <g>
                    {robots.map((robot) => (
                        <g key={robot.id}>
                            {robot.online && (
                                <>
                                    {/* Simulated 6m P2P communication radius (6m / 0.05m = 120px) */}
                                    <circle
                                        cx={robot.x}
                                        cy={robot.y}
                                        r="120"
                                        fill={robot.colorDim}
                                        fillOpacity=".18"
                                        stroke={robot.color}
                                        strokeDasharray="3 4"
                                        strokeWidth="1"
                                        strokeOpacity=".55"
                                    />
                                    <circle
                                        className="origin-center animate-[pulse_3s_ease-out_infinite]"
                                        cx={robot.x}
                                        cy={robot.y}
                                        r="120"
                                        fill="none"
                                        stroke={robot.color}
                                        strokeDasharray="3 4"
                                        strokeWidth="1"
                                        strokeOpacity=".35"
                                    />
                                </>
                            )}
                            {/* Heading arrow if angle is known */}
                            {robot.angleDeg !== undefined && (
                                <polygon
                                    points="0,-12 4.5,0 -4.5,0"
                                    fill={robot.color}
                                    stroke="white"
                                    strokeWidth="1"
                                    transform={`translate(${robot.x}, ${robot.y}) rotate(${robot.angleDeg})`}
                                />
                            )}
                            {/* Robot chassis circle (~70cm diameter = 7px radius at 0.05m/px) */}
                            <circle
                                cx={robot.x}
                                cy={robot.y}
                                r="7"
                                fill={robot.online ? robot.color : colors.offline}
                                stroke="white"
                                strokeWidth="2"
                            />
                            {/* Robot ID label */}
                            <text
                                x={robot.x + 10}
                                y={robot.y - 4}
                                fill="#1b231f"
                                fontFamily="IBM Plex Sans, sans-serif"
                                fontSize="9.5"
                                fontWeight="700"
                                style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                            >
                                {robot.id}
                            </text>
                            <text
                                x={robot.x + 10}
                                y={robot.y + 7}
                                fill="#4a554e"
                                fontFamily="IBM Plex Sans, sans-serif"
                                fontSize="8"
                                fontWeight="600"
                                style={{ filter: "drop-shadow(0px 1px 2px rgba(255,255,255,0.95))" }}
                            >
                                {robot.online
                                    ? robot.state === "warn"
                                        ? "low battery"
                                        : `${robot.battery}% • ${robot.task && robot.task !== '—' ? robot.task : 'idle'}`
                                    : "offline"}
                            </text>
                        </g>
                    ))}
                </g>
            </svg>

            {/* Broadcaster RF Status Tag */}
            {dockInfo && (
                <div className="absolute left-4 top-4 z-[6] flex items-center gap-2 rounded-[8px] border border-[#f59e0b]/40 bg-white/95 px-3 py-1.5 text-[11.5px] font-semibold text-[#92400e] shadow-sm backdrop-blur-sm">
                    <span className="relative flex h-2.5 w-2.5">
                        <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-[#f59e0b] opacity-75"></span>
                        <span className="relative inline-flex h-2.5 w-2.5 rounded-full bg-[#d97706]"></span>
                    </span>
                    <span>Broadcaster: ({dockInfo.rosX?.toFixed(1)}, {dockInfo.rosY?.toFixed(1)})</span>
                    <span className="rounded bg-[#fef3c7] px-1.5 py-0.5 text-[10px] font-bold text-[#b45309] border border-[#fcd34d]">
                        {dockInfo.radiusMeters}m RF Range
                    </span>
                </div>
            )}

            {/* Top-Right Quick Aisle Controls */}
            <div className="absolute top-4 right-4 z-[6] flex items-center gap-2">
                {aisleDrawingMode ? (
                    <button
                        onClick={onCancelAisleDraw}
                        className="flex items-center gap-1 rounded-[8px] border border-[#c0453b]/40 bg-white/95 px-3 py-1.5 text-xs font-bold text-[#c0453b] shadow-sm backdrop-blur-sm hover:bg-[#f6e4e1] transition-colors"
                    >
                        ✕ Cancel Draw
                    </button>
                ) : (
                    <button
                        onClick={onOpenAisleManager}
                        className="flex items-center gap-1.5 rounded-[8px] border border-[#6366f1]/30 bg-white/95 px-3 py-1.5 text-xs font-bold text-[#4338ca] shadow-sm backdrop-blur-sm hover:bg-[#eef2ff] transition-colors"
                        title="View & configure single-lane aisle reservation zones"
                    >
                        <span>🛣️</span>
                        <span>Aisles ({aisles.length})</span>
                    </button>
                )}
            </div>

            {aisleDrawingMode && (
                <div className="absolute left-1/2 top-4 z-[7] -translate-x-1/2 flex items-center gap-3 rounded-[9px] border border-[#6366f1] bg-[#1e1b4b]/95 px-4 py-2 text-xs font-semibold text-white shadow-lg backdrop-blur-sm">
                    <span className="h-2 w-2 rounded-full bg-[#818cf8] animate-ping" />
                    <span>
                        {aisleCorners.length === 0
                            ? "Click Map: Set Corner 1 of single-lane corridor"
                            : "Click Map: Set opposite Corner 2 to finish bounds"}
                    </span>
                    <button
                        onClick={onCancelAisleDraw}
                        className="ml-1 rounded bg-white/20 px-2 py-0.5 text-[11px] font-bold text-white hover:bg-white/30"
                    >
                        Cancel
                    </button>
                </div>
            )}

            {selectionMode && (
                <div ref={instructionRef} className={`pointer-events-none absolute left-1/2 top-4 z-[6] -translate-x-1/2 rounded-[9px] border border-[#b9d8c9] bg-white px-3.5 py-2 text-center text-xs font-semibold text-[#2f6f5e] shadow-[0_2px_8px_rgb(27_35_31_/_8%)] transition-opacity duration-150 ${instructionHovered ? "bg-white/75 opacity-80" : "opacity-100"}`}>
                    {selectedPoints.length === 0 ? `Click map to set Task ${currentTaskNum} PICKUP` : `Click map to set Task ${currentTaskNum} DROPOFF`}
                </div>
            )}
            {blockedMessage && (
                <div className="pointer-events-none fixed bottom-4 right-4 z-50 flex max-w-[min(360px,calc(100vw-2rem))] items-center gap-2.5 animate-[map-toast-in-out-right_3.2s_ease-in-out_forwards] rounded-[10px] border-2 border-[#c0453b] bg-[#fff5f3] px-4 py-3 text-left text-[13px] font-semibold text-[#a9362f] shadow-[0_6px_22px_rgb(192_69_59_/_30%)]">
                    <span className="flex h-6 w-6 flex-none items-center justify-center rounded-full bg-[#c0453b] text-sm font-bold text-white">!</span>
                    {blockedMessage}
                </div>
            )}

            <div className="absolute bottom-4 right-4 z-[6] flex flex-col overflow-hidden rounded-[9px] border border-[#e3e6e1] bg-white shadow-[0_2px_8px_rgb(27_35_31_/_6%)]">
                <button
                    className="h-[34px] w-[34px] border-b border-[#eceee9] bg-white text-base text-[#1b231f] hover:bg-[#f7f8f6]"
                    aria-label="Zoom in"
                    onClick={() => zoomBy(0.8)}
                >
                    +
                </button>
                <button
                    className="h-[34px] w-[34px] border-b border-[#eceee9] bg-white text-base text-[#1b231f] hover:bg-[#f7f8f6]"
                    aria-label="Zoom out"
                    onClick={() => zoomBy(1.25)}
                >
                    −
                </button>
                <button
                    className="h-[34px] w-[34px] bg-white text-base text-[#1b231f] hover:bg-[#f7f8f6]"
                    aria-label="Reset map"
                    onClick={() => setViewBox(baseViewBox)}
                >
                    ⤾
                </button>
            </div>
        </section>
    );
}

function TaskStatusPanel({
    tasks,
    stagedTasks = [],
    selectionMode,
    onAdd,
    onCancel,
    onDelete,
    onBroadcastStaged,
    onRemoveStaged,
    onClearStaged,
}) {
    const onTimeCompletion = tasks.length
        ? Math.round(tasks.reduce((total, task) => total + task.progress, 0) / tasks.length)
        : 0;
    return (
        <section className="flex max-h-[360px] w-full flex-none flex-col border-t border-[#c4cbc5] bg-white md:max-h-none md:w-[280px] md:border-l md:border-t-0 md:border-[#c4cbc5] xl:w-[350px]">
            <div className="flex flex-none flex-wrap items-center justify-between gap-1.5 px-[16px] pb-2 pt-3">
                <h2 className="m-0 text-[14.5px] font-bold tracking-tight text-[#1b231f]" style={displayFont}>
                    TASK <i>STATUS</i>
                </h2>
                <div className="flex items-center gap-1.5">
                    <button
                        className="flex items-center gap-1 rounded-full border border-[#d6ded8] bg-[#f7f8f6] px-2.5 py-1 text-[11px] font-semibold text-[#4a554e] hover:bg-[#eceee9] transition-colors"
                        onClick={onStageFive}
                        title="Quick-fill 5 warehouse tasks"
                    >
                        <span>⚡</span>
                        <span>5 Preset</span>
                    </button>
                    <button
                        className={`relative flex h-6 items-center justify-start rounded-full pl-6 pr-2.5 text-[11px] font-semibold transition-colors ${selectionMode
                            ? "bg-[#f6e4e1] text-[#c0453b] hover:bg-[#efc7c2]"
                            : "bg-[#e4efe9] text-[#2f6f5e] hover:bg-[#d9ecdf]"
                            }`}
                        onClick={selectionMode ? onCancel : onAdd}
                        aria-label={selectionMode ? "Cancel staging" : "Add task via map"}
                    >
                        <span className={`absolute left-2.5 top-1/2 inline-block w-3 -translate-y-1/2 text-center text-xs leading-none transition-transform duration-200 ${selectionMode ? "rotate-45" : "rotate-0"}`}>
                            +
                        </span>
                        {selectionMode ? "Cancel" : "Add Task"}
                    </button>
                </div>
            </div>

            {/* STAGED TASKS BATCH (Ready to broadcast all at once) */}
            {stagedTasks.length > 0 && (
                <div className="mx-3.5 mb-2 rounded-[9px] border border-[#2f6f5e]/30 bg-[#eef6f2] p-2.5 shadow-xs">
                    <div className="flex items-center justify-between pb-1.5 border-b border-[#2f6f5e]/15">
                        <span className="flex items-center gap-1.5 text-[11.5px] font-bold text-[#2f6f5e]">
                            <span className="inline-block h-2 w-2 rounded-full bg-[#2f6f5e] animate-pulse" />
                            STAGED BATCH
                        </span>
                        <button
                            onClick={onClearStaged}
                            className="text-[10.5px] font-medium text-[#8e988f] hover:text-[#c0453b]"
                        >
                            Clear all
                        </button>
                    </div>
                    <div className="my-2 max-h-32 space-y-1.5 overflow-y-auto pr-0.5">
                        {stagedTasks.map((st) => (
                            <div
                                key={st.id}
                                className="flex items-center justify-between gap-1.5 rounded-[6px] border border-[#dce8e0] bg-white px-2 py-1 text-[11px]"
                            >
                                <span className="font-bold text-[#1b231f]">{st.name}</span>
                                <span className="truncate text-[10px] text-[#6b776f]">
                                    P: ({st.pickupRos?.x?.toFixed(1) ?? '?'}, {st.pickupRos?.y?.toFixed(1) ?? '?'})m → D: ({st.dropoffRos?.x?.toFixed(1) ?? '?'}, {st.dropoffRos?.y?.toFixed(1) ?? '?'})m
                                </span>
                                <button
                                    onClick={() => onRemoveStaged(st.id)}
                                    className="flex h-4 w-4 flex-none items-center justify-center rounded text-xs text-[#8e988f] hover:bg-[#f6e4e1] hover:text-[#c0453b]"
                                    title="Remove task from staged batch"
                                >
                                    ×
                                </button>
                            </div>
                        ))}
                    </div>
                    <button
                        onClick={onBroadcastStaged}
                        className="w-full flex items-center justify-center gap-1.5 rounded-[7px] bg-[#2f6f5e] py-1.5 text-xs font-bold text-white shadow-sm hover:bg-[#265b4d] active:scale-[0.99] transition-all"
                    >
                        <span></span>
                        <span>Broadcast Fleet Tasks ({stagedTasks.length})</span>
                    </button>
                </div>
            )}

            <div className="border-b border-[#eceee9] px-[16px] pb-2 pt-1 flex items-baseline justify-between">
                <span className="text-[12px] font-semibold text-[#6b776f]">Fleet Task Pool</span>
                <div className="flex items-baseline gap-1.5">
                    <span className="font-bold text-[13px] text-[#2f6f5e]">{onTimeCompletion}%</span>
                    <span className="text-[10.5px] text-[#8e988f]">completion</span>
                </div>
            </div>

            <div className="flex-1 overflow-y-auto px-3.5 py-2">
                {tasks.length === 0 && (
                    <div className="py-6 text-center text-xs text-[#8e988f]">
                        No active tasks in pool. Stage tasks above and click <b>Broadcast</b>.
                    </div>
                )}
                {[...tasks].reverse().map((task, index) => (
                    <div
                        className="mb-2 rounded-[9px] border border-[#eceee9] bg-[#f7f8f6] p-[10px_11px]"
                        key={`${task.name}-${index}`}
                    >
                        <div className="mb-1.5 flex items-center justify-between gap-2">
                            <div className="text-[12.5px] font-bold text-[#1b231f]">{task.name}</div>
                            <div className="flex items-center gap-1.5">
                                <div
                                    className={`whitespace-nowrap rounded-full px-2 py-0.5 text-[10px] font-semibold ${task.state === 3 ? 'bg-[#e4efe9] text-[#2f6f5e]'
                                        : task.state === 2 || task.state === 6 || task.state === 7 ? 'bg-[#fdf3e4] text-[#c97a2b]'
                                            : task.assigned ? 'bg-[#e8e8f7] text-[#5a50a0]'
                                                : 'bg-[#eceee9] text-[#6b776f]'
                                        }`}
                                >
                                    {task.stateLabel || (task.assigned ? 'Assigned' : 'Available')}
                                </div>
                                {!task.assigned && (
                                    <button
                                        className="flex h-5 w-5 items-center justify-center rounded-[5px] border border-[#e3e6e1] bg-white text-[#c0453b] hover:border-[#efc7c2] hover:bg-[#f6e4e1]"
                                        aria-label={`Delete ${task.name}`}
                                        title={`Delete ${task.name}`}
                                        onClick={() => onDelete(task.number)}
                                    >
                                        <svg viewBox="0 0 24 24" className="h-3 w-3" fill="none" aria-hidden="true">
                                            <path d="M4 7h16M10 11v6M14 11v6M6 7l1 13h10l1-13M9 7l1-3h4l1 3" stroke="currentColor" strokeWidth="1.8" strokeLinecap="round" strokeLinejoin="round" />
                                        </svg>
                                    </button>
                                )}
                            </div>
                        </div>
                        <div className="mb-1.5 text-[11px] text-[#6b776f]">
                            {task.assigned ? (
                                <>Assigned to <b className="text-[#1b231f]">{task.robot}</b></>
                            ) : (
                                "Available in pool"
                            )}
                        </div>
                        <div className="h-[4px] overflow-hidden rounded-[2px] bg-[#eceee9]">
                            <div
                                className="h-full rounded-[2px] bg-[#2f6f5e]"
                                style={{ width: `${task.progress}%` }}
                            />
                        </div>
                    </div>
                ))}
            </div>
        </section>
    );
}

function DeleteTaskDialog({ task, onCancel, onConfirm }) {
    useEffect(() => {
        function handleKeyDown(event) {
            if (event.key === "Escape") onCancel();
        }
        window.addEventListener("keydown", handleKeyDown);
        return () => window.removeEventListener("keydown", handleKeyDown);
    }, [onCancel]);

    return (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-[#1b231f]/35 p-4">
            <div className="w-full max-w-sm rounded-xl bg-white p-5 shadow-[0_12px_32px_rgb(27_35_31_/_18%)]">
                <h3 className="text-base font-semibold" style={displayFont}>Delete task?</h3>
                <p className="mt-2 text-[13px] text-[#6b776f]">
                    Delete <b className="text-[#1b231f]">{task.name}</b> from the queue?
                </p>
                <div className="mt-5 flex justify-end gap-2">
                    <button
                        className="rounded-[7px] border border-[#e3e6e1] px-3.5 py-2 text-xs font-semibold text-[#6b776f] hover:bg-[#f7f8f6]"
                        onClick={onCancel}
                    >
                        Cancel
                    </button>
                    <button
                        className="rounded-[7px] bg-[#c0453b] px-3.5 py-2 text-xs font-semibold text-white hover:bg-[#a93c33]"
                        onClick={onConfirm}
                        autoFocus
                    >
                        Delete
                    </button>
                </div>
            </div>
        </div>
    );
}

function getTimelineRows(tasks) {
    return tasks.map((task, index) => ({
        label: task.name,
        bars: [{
            start: 30 + index * 75,
            end: 90 + index * 75,
            progress: task.progress,
            status: task.assigned ? "active" : "scheduled",
        }],
    }));
}

function formatDecision(event) {
    const ownership = event.ownership || {};
    const assignments = Object.entries(ownership)
        .map(([task, robot]) => `${task} → ${robot}`)
        .join(", ");
    if (assignments) return assignments;
    if (event.task_id) return `${event.task_id} → ${event.winner || event.robot_id || "pending"}`;
    if (event.event === "allocation_skipped") return "No eligible tasks or robots";
    if (event.event === "join_waiting_for_state_sync") return "Waiting for peer state sync";
    return event.event?.replaceAll("_", " ") || "Allocator update";
}

function formatTrafficEvent(event) {
    const segment = event.segment_id ? ` · ${event.segment_id}` : "";
    if (event.event === "reservation_granted") return `Aisle granted${segment}`;
    if (event.event === "reservation_queued") return `Waiting for aisle${segment}`;
    if (event.event === "reservation_promoted") return `Aisle access promoted${segment}`;
    if (event.event === "reservation_released") return `Aisle released${segment}`;
    if (event.event === "reservation_expired") return `Stale aisle lease expired${segment}`;
    if (event.event === "reservation_requested") return `Aisle requested${segment}`;
    if (event.event === "orca_avoidance") return "ORCA collision avoidance active";
    if (event.event === "pibt_wait") return `PIBT yielding at choke point${segment}`;
    return event.event?.replaceAll("_", " ") || "Traffic update";
}

function AllocationPanel({ allocationEvents, trafficEvents, liveBundles }) {
    const relevantEvents = allocationEvents.filter((event) => [
        "allocation_decided",
        "binary_round_decided",
        "allocation_skipped",
        "join_waiting_for_state_sync",
    ].includes(event.event)).slice(-5).reverse();
    const recentTraffic = trafficEvents.slice(-5).reverse();
    const bundles = Object.entries(liveBundles).sort(([a], [b]) => a.localeCompare(b));

    return (
        <section className="flex min-h-0 flex-[0.8] flex-col border-t border-[#c4cbc5] bg-white md:border-l">
            <div className="flex items-center justify-between border-b border-[#eceee9] px-3.5 py-2.5 md:px-5">
                <h2 className="m-0 text-[13px] font-semibold" style={displayFont}>
                    ALLOCATION <i>TRACE</i>
                </h2>
                <span className="text-[10.5px] text-[#8e988f]">Live ROS audit</span>
            </div>
            <div className="grid min-h-0 flex-1 grid-cols-1 divide-y divide-[#eceee9] overflow-y-auto md:grid-cols-2 md:divide-x md:divide-y-0">
                <div className="p-3.5 md:p-4">
                    <div className="mb-2 text-[10.5px] font-bold tracking-wide text-[#6b776f]">RECENT DECISIONS</div>
                    {relevantEvents.length === 0 ? (
                        <p className="m-0 text-xs text-[#8e988f]">Waiting for a fleet allocation event.</p>
                    ) : (
                        <div className="space-y-1.5">
                            {relevantEvents.map((event, index) => (
                                <div key={`${event.timestamp_utc || "event"}-${index}`} className="rounded-[7px] bg-[#f7f8f6] px-2.5 py-2">
                                    <div className="flex items-center justify-between gap-2 text-[10px]">
                                        <span className="font-bold uppercase text-[#2f6f5e]">{event.event?.replaceAll("_", " ")}</span>
                                        <span className="text-[#8e988f]">{event.robot_id || "fleet"}</span>
                                    </div>
                                    <div className="mt-0.5 text-[11.5px] font-medium text-[#1b231f]">{formatDecision(event)}</div>
                                </div>
                            ))}
                        </div>
                    )}
                    <div className="mb-2 mt-4 text-[10.5px] font-bold tracking-wide text-[#6b776f]">TRAFFIC CONTROL</div>
                    {recentTraffic.length === 0 ? (
                        <p className="m-0 text-xs text-[#8e988f]">No reservation, ORCA, or PIBT events yet.</p>
                    ) : (
                        <div className="space-y-1.5">
                            {recentTraffic.map((event, index) => {
                                const isAlert = ["reservation_queued", "reservation_expired", "pibt_wait", "orca_avoidance"].includes(event.event);
                                return (
                                    <div key={`${event.timestamp_utc || "traffic"}-${index}`} className="rounded-[7px] bg-[#f7f8f6] px-2.5 py-2">
                                        <div className="flex items-center justify-between gap-2 text-[10px]">
                                            <span className={`font-bold uppercase ${isAlert ? "text-[#c97a2b]" : "text-[#3978b7]"}`}>{event.event?.replaceAll("_", " ")}</span>
                                            <span className="text-[#8e988f]">{event.robot_id || "fleet"}</span>
                                        </div>
                                        <div className="mt-0.5 text-[11.5px] font-medium text-[#1b231f]">{formatTrafficEvent(event)}</div>
                                    </div>
                                );
                            })}
                        </div>
                    )}
                </div>
                <div className="p-3.5 md:p-4">
                    <div className="mb-2 text-[10.5px] font-bold tracking-wide text-[#6b776f]">ROBOT BUNDLES</div>
                    {bundles.length === 0 ? (
                        <p className="m-0 text-xs text-[#8e988f]">No bundles published yet.</p>
                    ) : (
                        <div className="space-y-1.5">
                            {bundles.map(([robotId, bundle]) => (
                                <div key={robotId} className="flex items-center gap-2 rounded-[7px] bg-[#f7f8f6] px-2.5 py-2 text-[11.5px]">
                                    <span className="font-bold text-[#3978b7]">{robotId}</span>
                                    <span className="min-w-0 truncate text-[#4a554e]">{bundle.task_ids?.length ? bundle.task_ids.join(" → ") : "Idle"}</span>
                                </div>
                            ))}
                        </div>
                    )}
                </div>
            </div>
        </section>
    );
}

function MapScale({ mapScale }) {
    const scaleOptions = [0.5, 1, 2, 5, 10, 20, 50];
    const zoom = mapScale?.zoom ?? 100;
    const pixelsPerMeter = mapScale?.pixelsPerMeter ?? 20;
    const distance = [...scaleOptions].reverse().find((option) => option * pixelsPerMeter <= 130) || 0.5;
    const width = Math.max(24, Math.round(distance * pixelsPerMeter));

    return (
        <div className="flex items-center gap-2 rounded-[7px] border border-[#e3e6e1] bg-[#f7f8f6] px-2.5 py-1 text-[10.5px] font-medium text-[#4a554e]" aria-label={`Map scale: ${distance} meters at ${zoom}% zoom`}>
            <span className="text-[10px] font-semibold uppercase tracking-[0.08em] text-[#8e988f]">Scale</span>
            <div className="relative h-2 border-b-2 border-l-2 border-r-2 border-[#2f6f5e]" style={{ width }} />
            <span className="whitespace-nowrap">{distance} m</span>
            <span className="text-[#8e988f]">{zoom}%</span>
        </div>
    );
}

function EfficiencyPanel({ tasks, mapZoom }) {
    const [fullScreen, setFullScreen] = useState(false);
    const [minimized, setMinimized] = useState(false);

    return (
        <section
            className={`flex flex-none flex-col border-t border-[#c4cbc5] bg-white ${fullScreen ? "fixed inset-0 z-40 h-screen" : minimized ? "h-9" : "h-[270px] md:h-[248px]"}`}
        >
            <div className="relative flex h-9 flex-none items-center justify-between border-b border-[#eceee9] px-3.5 md:px-5">
                <h2 className="m-0 whitespace-nowrap text-[13px] font-semibold" style={displayFont}>
                    EFFICIENCY <i>TIMELINE</i>
                </h2>
                <div className="absolute left-1/2 -translate-x-1/2">
                    {!minimized && !fullScreen && <MapScale mapScale={mapZoom} />}
                </div>
                <div className="flex items-center gap-1">
                    <button
                        className="relative flex h-6 w-6 items-center justify-center rounded-full text-[#6b776f] hover:bg-[#f7f8f6]"
                        aria-label={fullScreen ? "Exit fullscreen chart" : "View chart fullscreen"}
                        onClick={() => {
                            setFullScreen((current) => !current);
                            setMinimized(false);
                        }}
                    >
                        <svg
                            className="h-5 w-5"
                            viewBox="0 0 24 24"
                            fill="none"
                            xmlns="http://www.w3.org/2000/svg"
                            aria-hidden="true"
                        >
                            <path
                                d={
                                    fullScreen
                                        ? "M9.00001 18.0001L9.00001 17.0001C9.00001 15.8956 8.10458 15.0001 7.00001 15.0001H6.00001M15 18.0001V17.0001C15 15.8956 15.8954 15.0001 17 15.0001L18 15.0001M9 6.00012L9 7.00012C9 8.10469 8.10457 9.00012 7 9.00012L6 9.00012M15 6.00014L15 7.00014C15 8.10471 15.8954 9.00014 17 9.00014L18 9.00014"
                                        : "M6 15V16C6 17.1046 6.89543 18 8 18H9M18 15V16C18 17.1046 17.1046 18 16 18H15M6 9V8C6 6.89543 6.89543 6 8 6H9M18 9V8C18 6.89543 17.1046 6 16 6H15"
                                }
                                stroke="currentColor"
                                strokeWidth="2"
                                strokeLinecap="round"
                                strokeLinejoin="round"
                            />
                        </svg>
                    </button>
                    {!fullScreen && (
                        <button
                            className="flex h-6 w-6 items-center justify-center rounded-full text-[#6b776f] hover:bg-[#f7f8f6]"
                            aria-label={minimized ? "Show chart" : "Minimize chart"}
                            title={minimized ? "Show timeline" : "Minimize timeline"}
                            onClick={() => {
                                setMinimized((current) => !current);
                                setFullScreen(false);
                            }}
                        >
                            <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
                                <path
                                    d={minimized ? "m6 14 6-6 6 6" : "m6 10 6 6 6-6"}
                                    stroke="currentColor"
                                    strokeWidth="2"
                                    strokeLinecap="round"
                                    strokeLinejoin="round"
                                />
                            </svg>
                        </button>
                    )}
                </div>
            </div>
            {!minimized && (
                <div className="min-h-0 flex-1 overflow-auto px-3.5 pb-3.5 md:px-5">
                    <div className="flex min-w-[640px]">
                        <div className="sticky left-0 z-[3] w-[140px] flex-none bg-white pt-[34px] md:w-[180px]">
                            {getTimelineRows(tasks).map((row) => (
                                <div
                                    className="flex h-[30px] items-center gap-1 text-xs font-medium"
                                    key={row.label}
                                >
                                    <span>{row.label}</span>
                                    <span
                                        className={`whitespace-nowrap rounded-full px-1.5 py-0.5 text-[10px] font-medium ${row.bars[0].status === "delayed" ? "bg-[#f6e4e1] text-[#c0453b]" : "bg-[#e4efe9] text-[#2f6f5e]"}`}
                                    >
                                        {row.bars[0].status === "delayed"
                                            ? "Delayed"
                                            : row.bars[0].status === "done"
                                                ? "Completed"
                                                : row.bars[0].status === "scheduled"
                                                    ? "Unassigned"
                                                    : "In progress"}
                                    </span>
                                </div>
                            ))}
                        </div>
                        <div className="relative min-w-0 flex-1">
                            <div className="relative h-[34px] border-b border-[#eceee9]">
                                {Array.from({ length: 11 }, (_, index) => (
                                    <div
                                        className={`absolute top-2 text-[10.5px] text-[#8e988f] ${index === 0 ? "" : index === 10 ? "-translate-x-full" : "-translate-x-1/2"}`}
                                        style={{ left: `${index * 10}%` }}
                                        key={index}
                                    >{`${8 + index}:00`}</div>
                                ))}
                            </div>
                            {getTimelineRows(tasks).map((row) => (
                                <div
                                    className="relative h-[30px] border-b border-[#eceee9]"
                                    key={row.label}
                                >
                                    {row.bars.map((bar) => (
                                        <div
                                            className={`absolute top-1.5 h-[18px] overflow-hidden rounded-[5px] ${bar.status === "scheduled" ? "border border-dashed border-[#8e988f] bg-transparent" : bar.status === "delayed" ? "border border-[#c0453b] bg-[#f6e4e1]" : "border border-[#2f6f5e] bg-[#e4efe9]"}`}
                                            style={{
                                                left: `${(bar.start / axisEnd) * 100}%`,
                                                width: `${((bar.end - bar.start) / axisEnd) * 100}%`,
                                            }}
                                            key={`${row.label}-${bar.start}`}
                                        >
                                            {bar.status !== "scheduled" && (
                                                <div
                                                    className={`h-full opacity-85 ${bar.status === "delayed" ? "bg-[#c0453b]" : "bg-[#2f6f5e]"}`}
                                                    style={{ width: `${bar.progress}%` }}
                                                />
                                            )}
                                        </div>
                                    ))}
                                </div>
                            ))}
                            <div
                                className="absolute top-0 z-[2] w-[1.5px] bg-[#c97a2b]"
                                style={{
                                    left: `${(now / axisEnd) * 100}%`,
                                    height: `${getTimelineRows(tasks).length * 30 + 34}px`,
                                }}
                            >
                                <span className="absolute -top-[18px] left-1 text-[10px] font-semibold text-[#c97a2b]">
                                    now
                                </span>
                            </div>
                        </div>
                    </div>
                </div>
            )}
        </section>
    );
}

function AddTaskModal({ robots = [], onClose, onConfirm }) {
    const [name, setName] = useState("");
    const [robot, setRobot] = useState("");
    return (
        <div
            className="fixed inset-0 z-50 flex items-center justify-center bg-[#1b231f]/35"
            onClick={(event) => event.target === event.currentTarget && onClose()}
        >
            <div className="w-80 rounded-xl bg-white p-[22px] shadow-[0_12px_32px_rgb(27_35_31_/_18%)]">
                <h3 className="mb-4 text-base font-semibold" style={displayFont}>
                    Add task
                </h3>
                <div className="mb-3.5">
                    <label
                        className="mb-1 block text-[11.5px] font-medium text-[#6b776f]"
                        htmlFor="new-task-name"
                    >
                        Task name
                    </label>
                    <input
                        className="w-full rounded-[7px] border border-[#e3e6e1] bg-[#f7f8f6] px-2.5 py-2 text-[13px] outline-none focus:outline-2 focus:outline-[#2f6f5e]"
                        id="new-task-name"
                        value={name}
                        onChange={(event) => setName(event.target.value)}
                        placeholder="e.g. Restock — Aisle 9"
                        autoFocus
                    />
                </div>
                <div className="mb-3.5">
                    <label
                        className="mb-1 block text-[11.5px] font-medium text-[#6b776f]"
                        htmlFor="new-task-robot"
                    >
                        Assign to
                    </label>
                    <select
                        className="w-full rounded-[7px] border border-[#e3e6e1] bg-[#f7f8f6] px-2.5 py-2 text-[13px] outline-none focus:outline-2 focus:outline-[#2f6f5e]"
                        id="new-task-robot"
                        value={robot}
                        onChange={(event) => setRobot(event.target.value)}
                    >
                        <option value="">Leave unassigned</option>
                        {robots
                            .filter((item) => item.online)
                            .map((item) => (
                                <option key={item.id} value={item.id}>
                                    {item.id}
                                </option>
                            ))}
                    </select>
                </div>
                <div className="mt-[18px] flex justify-end gap-2">
                    <button
                        className="rounded-[7px] border border-[#e3e6e1] px-3.5 py-2 text-xs font-semibold text-[#6b776f]"
                        onClick={onClose}
                    >
                        Cancel
                    </button>
                    <button
                        className="rounded-[7px] bg-[#2f6f5e] px-3.5 py-2 text-xs font-semibold text-white hover:bg-[#28604f]"
                        onClick={() =>
                            name.trim() &&
                            onConfirm({
                                name: name.trim(),
                                assigned: Boolean(robot),
                                robot: robot || null,
                                progress: 0,
                            })
                        }
                    >
                        Add task
                    </button>
                </div>
            </div>
        </div>
    );
}

function AisleEditModal({ aisle, isNew = false, onClose, onSave }) {
    const [name, setName] = useState(aisle?.name || "");
    const [segId, setSegId] = useState(aisle?.segment_id || aisle?.id || "");
    const [xMin, setXMin] = useState(aisle?.x_min !== undefined ? aisle.x_min.toString() : "0.0");
    const [xMax, setXMax] = useState(aisle?.x_max !== undefined ? aisle.x_max.toString() : "2.0");
    const [yMin, setYMin] = useState(aisle?.y_min !== undefined ? aisle.y_min.toString() : "0.0");
    const [yMax, setYMax] = useState(aisle?.y_max !== undefined ? aisle.y_max.toString() : "2.0");
    const [isSingleLane, setIsSingleLane] = useState(aisle?.is_single_lane ?? true);
    const [error, setError] = useState("");

    const handleSubmit = (e) => {
        e.preventDefault();
        const numX1 = parseFloat(xMin);
        const numX2 = parseFloat(xMax);
        const numY1 = parseFloat(yMin);
        const numY2 = parseFloat(yMax);

        if (isNaN(numX1) || isNaN(numX2) || isNaN(numY1) || isNaN(numY2)) {
            setError("All bounds must be valid floating point numbers in meters.");
            return;
        }

        const actualXMin = Math.min(numX1, numX2);
        const actualXMax = Math.max(numX1, numX2);
        const actualYMin = Math.min(numY1, numY2);
        const actualYMax = Math.max(numY1, numY2);

        if (actualXMax - actualXMin < 0.2 || actualYMax - actualYMin < 0.2) {
            setError("Corridor must be at least 0.2m × 0.2m in dimension.");
            return;
        }

        const finalId = (segId.trim() || `aisle_${Date.now().toString().slice(-4)}`).toLowerCase().replace(/\s+/g, "_");
        const finalName = name.trim() || finalId;

        onSave({
            id: finalId,
            segment_id: finalId,
            name: finalName,
            x_min: Math.round(actualXMin * 1000) / 1000,
            x_max: Math.round(actualXMax * 1000) / 1000,
            y_min: Math.round(actualYMin * 1000) / 1000,
            y_max: Math.round(actualYMax * 1000) / 1000,
            is_single_lane: isSingleLane,
        });
    };

    return (
        <div
            className="fixed inset-0 z-50 flex items-center justify-center bg-[#1b231f]/40 backdrop-blur-[2px] p-4"
            onClick={(e) => e.target === e.currentTarget && onClose()}
        >
            <div className="w-full max-w-md rounded-2xl bg-white p-6 shadow-2xl border border-[#e3e6e1]">
                <div className="flex items-center justify-between pb-3 border-b border-[#eceee9]">
                    <div className="flex items-center gap-2">
                        <span className="flex h-7 w-7 items-center justify-center rounded-lg bg-[#eef2ff] text-[#4338ca] text-sm font-bold">
                            🛣️
                        </span>
                        <h3 className="text-base font-bold text-[#1b231f]" style={displayFont}>
                            {isNew ? "Define New Single-Lane Aisle" : "Edit Aisle Configuration"}
                        </h3>
                    </div>
                    <button
                        onClick={onClose}
                        className="rounded-lg p-1 text-[#8e988f] hover:bg-[#f7f8f6] hover:text-[#1b231f]"
                    >
                        ✕
                    </button>
                </div>

                <form onSubmit={handleSubmit} className="mt-4 space-y-4">
                    {error && (
                        <div className="rounded-lg border border-[#c0453b]/30 bg-[#fff5f3] px-3 py-2 text-xs font-semibold text-[#c0453b]">
                            {error}
                        </div>
                    )}

                    <div className="grid grid-cols-2 gap-3">
                        <div>
                            <label className="mb-1 block text-[11px] font-semibold text-[#6b776f]">
                                Display Name
                            </label>
                            <input
                                className="w-full rounded-lg border border-[#e3e6e1] bg-[#f7f8f6] px-3 py-2 text-xs font-medium text-[#1b231f] outline-none focus:border-[#6366f1] focus:bg-white transition-colors"
                                value={name}
                                onChange={(e) => setName(e.target.value)}
                                placeholder="e.g. Aisle 1"
                                autoFocus
                            />
                        </div>
                        <div>
                            <label className="mb-1 block text-[11px] font-semibold text-[#6b776f]">
                                Segment ID
                            </label>
                            <input
                                className="w-full rounded-lg border border-[#e3e6e1] bg-[#f7f8f6] px-3 py-2 text-xs font-mono font-medium text-[#1b231f] outline-none focus:border-[#6366f1] focus:bg-white transition-colors"
                                value={segId}
                                onChange={(e) => setSegId(e.target.value)}
                                placeholder="e.g. aisle_1"
                            />
                        </div>
                    </div>

                    <div className="rounded-xl border border-[#eceee9] bg-[#f9fafb] p-3.5 space-y-3">
                        <div className="flex items-center justify-between">
                            <span className="text-xs font-bold text-[#374151]">ROS Map Metric Coordinates (Meters)</span>
                            <span className="text-[10px] text-[#6b776f]">logistics_warehouse frame</span>
                        </div>

                        <div className="grid grid-cols-2 gap-2.5">
                            <div>
                                <label className="mb-1 block text-[10.5px] font-medium text-[#6b776f]">
                                    X Min (m)
                                </label>
                                <input
                                    type="number"
                                    step="0.05"
                                    className="w-full rounded-lg border border-[#e3e6e1] bg-white px-2.5 py-1.5 text-xs font-mono text-[#1b231f] outline-none focus:border-[#6366f1]"
                                    value={xMin}
                                    onChange={(e) => setXMin(e.target.value)}
                                />
                            </div>
                            <div>
                                <label className="mb-1 block text-[10.5px] font-medium text-[#6b776f]">
                                    X Max (m)
                                </label>
                                <input
                                    type="number"
                                    step="0.05"
                                    className="w-full rounded-lg border border-[#e3e6e1] bg-white px-2.5 py-1.5 text-xs font-mono text-[#1b231f] outline-none focus:border-[#6366f1]"
                                    value={xMax}
                                    onChange={(e) => setXMax(e.target.value)}
                                />
                            </div>
                            <div>
                                <label className="mb-1 block text-[10.5px] font-medium text-[#6b776f]">
                                    Y Min (m)
                                </label>
                                <input
                                    type="number"
                                    step="0.05"
                                    className="w-full rounded-lg border border-[#e3e6e1] bg-white px-2.5 py-1.5 text-xs font-mono text-[#1b231f] outline-none focus:border-[#6366f1]"
                                    value={yMin}
                                    onChange={(e) => setYMin(e.target.value)}
                                />
                            </div>
                            <div>
                                <label className="mb-1 block text-[10.5px] font-medium text-[#6b776f]">
                                    Y Max (m)
                                </label>
                                <input
                                    type="number"
                                    step="0.05"
                                    className="w-full rounded-lg border border-[#e3e6e1] bg-white px-2.5 py-1.5 text-xs font-mono text-[#1b231f] outline-none focus:border-[#6366f1]"
                                    value={yMax}
                                    onChange={(e) => setYMax(e.target.value)}
                                />
                            </div>
                        </div>

                        <div className="flex items-center justify-between pt-1 text-[11px] text-[#4b5563] border-t border-[#eceee9]">
                            <span>Corridor Dimensions:</span>
                            <span className="font-mono font-semibold text-[#1f2937]">
                                ΔX = {Math.abs(parseFloat(xMax || 0) - parseFloat(xMin || 0)).toFixed(2)}m,
                                ΔY = {Math.abs(parseFloat(yMax || 0) - parseFloat(yMin || 0)).toFixed(2)}m
                            </span>
                        </div>
                    </div>

                    <div className="flex items-center gap-2.5 rounded-lg border border-[#eceee9] bg-[#f7f8f6] p-2.5">
                        <input
                            type="checkbox"
                            id="single-lane-toggle"
                            checked={isSingleLane}
                            onChange={(e) => setIsSingleLane(e.target.checked)}
                            className="h-4 w-4 rounded text-[#6366f1] focus:ring-[#6366f1]"
                        />
                        <label htmlFor="single-lane-toggle" className="text-xs font-semibold text-[#374151] cursor-pointer">
                            Enforce Single-Lane Mutual Exclusion (ORCA Disabled, PIBT Choke Arbitration Active)
                        </label>
                    </div>

                    <div className="flex items-center justify-end gap-2.5 pt-2 border-t border-[#eceee9]">
                        <button
                            type="button"
                            onClick={onClose}
                            className="rounded-xl border border-[#e3e6e1] px-4 py-2 text-xs font-semibold text-[#6b776f] hover:bg-[#f7f8f6] transition-colors"
                        >
                            Cancel
                        </button>
                        <button
                            type="submit"
                            className="rounded-xl bg-[#4338ca] px-4 py-2 text-xs font-semibold text-white shadow-sm hover:bg-[#3730a3] transition-colors"
                        >
                            {isNew ? "Add Aisle" : "Apply Changes"}
                        </button>
                    </div>
                </form>
            </div>
        </div>
    );
}

function AisleManagerDrawer({
    isOpen,
    onClose,
    aisles = [],
    selectedAisleId,
    onSelectAisle,
    onStartDrawAisle,
    onAddManualAisle,
    onEditAisle,
    onDeleteAisle,
    onResetDefaults,
    onSaveFleet,
    trafficEvents = [],
}) {
    if (!isOpen) return null;

    return (
        <div className="fixed inset-0 z-40 flex justify-end bg-black/30 backdrop-blur-[1px]">
            <div className="flex h-full w-full max-w-md flex-col bg-white shadow-2xl animate-[slideLeft_0.2s_ease-out]">
                {/* Drawer Header */}
                <div className="flex flex-none items-center justify-between border-b border-[#e3e6e1] px-5 py-3.5 bg-[#fcfdfc]">
                    <div className="flex items-center gap-2.5">
                        <span className="flex h-8 w-8 items-center justify-center rounded-xl bg-[#eef2ff] text-[#4338ca] text-base">
                            🛣️
                        </span>
                        <div>
                            <h2 className="text-[15px] font-bold text-[#1b231f]" style={displayFont}>
                                AISLE <i>MANAGER</i>
                            </h2>
                            <p className="text-[11px] text-[#6b776f]">
                                Mutual exclusion corridors & PIBT choke boundaries
                            </p>
                        </div>
                    </div>
                    <button
                        onClick={onClose}
                        className="rounded-lg p-1.5 text-[#8e988f] hover:bg-[#f7f8f6] hover:text-[#1b231f]"
                        aria-label="Close drawer"
                    >
                        ✕
                    </button>
                </div>

                {/* Action Bar */}
                <div className="flex flex-none flex-wrap items-center gap-2 border-b border-[#eceee9] bg-[#f9fafb] p-3">
                    <button
                        onClick={() => {
                            onClose();
                            onStartDrawAisle();
                        }}
                        className="flex items-center gap-1.5 rounded-lg border border-[#6366f1]/30 bg-white px-3 py-1.5 text-xs font-bold text-[#4338ca] shadow-xs hover:bg-[#eef2ff] transition-colors"
                        title="Click two opposite corners on the warehouse floor map to set bounds"
                    >
                        <span>✏️</span>
                        <span>Draw on Map</span>
                    </button>

                    <button
                        onClick={onAddManualAisle}
                        className="flex items-center gap-1.5 rounded-lg border border-[#d1d5db] bg-white px-3 py-1.5 text-xs font-semibold text-[#374151] shadow-xs hover:bg-[#f3f4f6] transition-colors"
                    >
                        <span>➕</span>
                        <span>Add Manually</span>
                    </button>

                    <button
                        onClick={onResetDefaults}
                        className="flex items-center gap-1 rounded-lg border border-[#e5e7eb] bg-white px-2.5 py-1.5 text-xs font-medium text-[#6b7280] shadow-xs hover:bg-[#f9fafb] hover:text-[#b91c1c] transition-colors"
                        title="Reset to 3 default warehouse aisles"
                    >
                        <span>↺</span>
                        <span>Reset Defaults</span>
                    </button>
                </div>

                {/* Aisle List */}
                <div className="flex-1 overflow-y-auto p-4 space-y-3">
                    {aisles.length === 0 ? (
                        <div className="rounded-xl border-2 border-dashed border-[#e3e6e1] p-6 text-center">
                            <span className="text-3xl">🛣️</span>
                            <p className="mt-2 text-xs font-semibold text-[#374151]">No single-lane aisles configured.</p>
                            <p className="mt-1 text-[11px] text-[#8e988f]">
                                Use "Draw on Map" or "Add Manually" to define mutual exclusion corridors.
                            </p>
                        </div>
                    ) : (
                        aisles.map((aisle) => {
                            const segId = aisle.segment_id || aisle.id;
                            const occ = getAisleOccupancy(segId, trafficEvents);
                            const isSelected = selectedAisleId === segId;
                            const dx = Math.abs(aisle.x_max - aisle.x_min).toFixed(2);
                            const dy = Math.abs(aisle.y_max - aisle.y_min).toFixed(2);

                            return (
                                <div
                                    key={segId}
                                    onClick={() => onSelectAisle(aisle)}
                                    className={`cursor-pointer rounded-xl border p-3.5 transition-all shadow-xs ${isSelected
                                            ? "border-[#6366f1] bg-[#f5f7ff] ring-1 ring-[#6366f1]"
                                            : "border-[#e3e6e1] bg-white hover:border-[#cbd5e1] hover:bg-[#fbfcfb]"
                                        }`}
                                >
                                    <div className="flex items-center justify-between pb-2 border-b border-[#eceee9]">
                                        <div className="flex items-center gap-2">
                                            <span className="font-bold text-xs text-[#1b231f]">{aisle.name || segId}</span>
                                            <span className="rounded bg-[#f3f4f6] px-1.5 py-0.5 font-mono text-[10px] text-[#4b5563]">
                                                {segId}
                                            </span>
                                        </div>
                                        <div className="flex items-center gap-1.5">
                                            {occ.occupied ? (
                                                <span className="flex items-center gap-1 rounded-full bg-[#fef3c7] border border-[#fcd34d] px-2 py-0.5 text-[10px] font-bold text-[#b45309]">
                                                    <span className="h-1.5 w-1.5 rounded-full bg-[#d97706] animate-pulse" />
                                                    Held by {occ.holder}
                                                </span>
                                            ) : (
                                                <span className="rounded-full bg-[#eef6f2] border border-[#b9d8c9] px-2 py-0.5 text-[10px] font-semibold text-[#2f6f5e]">
                                                    Clear (Open)
                                                </span>
                                            )}
                                        </div>
                                    </div>

                                    {/* Coordinate Readout */}
                                    <div className="my-2.5 grid grid-cols-2 gap-2 text-[11px] font-mono text-[#4b5563] bg-[#f9fafb] p-2 rounded-lg border border-[#f0f2f0]">
                                        <div>
                                            <span className="text-[#9ca3af]">X: </span>
                                            <span className="font-semibold text-[#1f2937]">[{aisle.x_min.toFixed(2)} → {aisle.x_max.toFixed(2)}]m</span>
                                            <div className="text-[10px] text-[#6b7280]">Span: {dx}m</div>
                                        </div>
                                        <div>
                                            <span className="text-[#9ca3af]">Y: </span>
                                            <span className="font-semibold text-[#1f2937]">[{aisle.y_min.toFixed(2)} → {aisle.y_max.toFixed(2)}]m</span>
                                            <div className="text-[10px] text-[#6b7280]">Span: {dy}m</div>
                                        </div>
                                    </div>

                                    <div className="flex items-center justify-between pt-1">
                                        <span className="text-[10.5px] text-[#6b776f] flex items-center gap-1">
                                            <span>🔒</span> Single-lane mutual exclusion
                                        </span>
                                        <div className="flex items-center gap-2">
                                            <button
                                                onClick={(e) => {
                                                    e.stopPropagation();
                                                    onEditAisle(aisle);
                                                }}
                                                className="rounded-md border border-[#e3e6e1] bg-white px-2 py-1 text-[11px] font-semibold text-[#4b5563] hover:bg-[#f3f4f6]"
                                            >
                                                Edit
                                            </button>
                                            <button
                                                onClick={(e) => {
                                                    e.stopPropagation();
                                                    onDeleteAisle(segId);
                                                }}
                                                className="rounded-md border border-[#fee2e2] bg-white px-2 py-1 text-[11px] font-semibold text-[#dc2626] hover:bg-[#fef2f2]"
                                            >
                                                Delete
                                            </button>
                                        </div>
                                    </div>
                                </div>
                            );
                        })
                    )}
                </div>

                {/* Footer / Broadcast Bar */}
                <div className="flex-none border-t border-[#e3e6e1] bg-[#f8fafc] p-4">
                    <div className="mb-2 flex items-center gap-2 text-[11px] text-[#64748b]">
                        <span>ℹ️</span>
                        <span>Persisted in `aisle_segments.json` & broadcasted on `/fleet/aisle_config`.</span>
                    </div>
                    <button
                        onClick={onSaveFleet}
                        className="w-full flex items-center justify-center gap-2 rounded-xl bg-[#2f6f5e] py-2.5 text-xs font-bold text-white shadow-md hover:bg-[#28604f] transition-colors"
                    >
                        <span>💾</span>
                        <span>Save & Broadcast to Fleet ({aisles.length} Aisles)</span>
                    </button>
                </div>
            </div>
        </div>
    );
}

function App() {
    // ── Live ROS data ──────────────────────────────────────────────────────
    const {
        rosStatus,
        liveRobots,
        liveTasks,
        liveBundles,
        allocationEvents,
        trafficEvents,
        liveAisles,
        dockInfo,
        broadcastTasks,
        saveAislesConfig,
    } = useRos();

    // Use live data when available, fall back to mock data
    const robots = liveRobots ?? [];

    // ── Task state (static + live merged) ─────────────────────────────────
    const [tasks, setTasks] = useState([]);
    // When live tasks arrive from ROS, replace the task list entirely
    useEffect(() => {
        if (liveTasks !== null) setTasks(liveTasks);
    }, [liveTasks]);

    // ── Single-lane Aisle state & controls ─────────────────────────────────
    const [aisles, setAisles] = useState(liveAisles ?? DEFAULT_AISLES);
    useEffect(() => {
        if (liveAisles && liveAisles.length > 0) setAisles(liveAisles);
    }, [liveAisles]);

    const [aisleDrawerOpen, setAisleDrawerOpen] = useState(false);
    const [aisleDrawingMode, setAisleDrawingMode] = useState(false);
    const [aisleCorners, setAisleCorners] = useState([]);
    const [editingAisle, setEditingAisle] = useState(null);
    const [selectedAisleId, setSelectedAisleId] = useState(null);

    // Keyboard shortcut (Escape cancels aisle drawing)
    useEffect(() => {
        function handleKeyDown(e) {
            if (e.key === "Escape") {
                if (aisleDrawingMode) {
                    setAisleDrawingMode(false);
                    setAisleCorners([]);
                    setToastMessage("Aisle drawing cancelled.");
                }
            }
        }
        window.addEventListener("keydown", handleKeyDown);
        return () => window.removeEventListener("keydown", handleKeyDown);
    }, [aisleDrawingMode]);

    function startAisleDraw() {
        setSelectionMode(false);
        setSelectedPoints([]);
        setAisleCorners([]);
        setAisleDrawingMode(true);
        setToastMessage("Click map: Select Corner 1 of the single-lane aisle.");
    }

    function cancelAisleDraw() {
        setAisleDrawingMode(false);
        setAisleCorners([]);
    }

    function handleAisleCornerSelect(point) {
        const corners = [...aisleCorners, point];
        setAisleCorners(corners);
        if (corners.length === 2) {
            const r1 = svgToRos(corners[0].x, corners[0].y);
            const r2 = svgToRos(corners[1].x, corners[1].y);
            const nextIndex = aisles.length + 1;
            const newAisle = {
                id: `aisle_${nextIndex}`,
                segment_id: `aisle_${nextIndex}`,
                name: `Aisle ${nextIndex}`,
                x_min: Math.min(r1.x, r2.x),
                x_max: Math.max(r1.x, r2.x),
                y_min: Math.min(r1.y, r2.y),
                y_max: Math.max(r1.y, r2.y),
                is_single_lane: true,
                isNew: true,
            };
            setAisleDrawingMode(false);
            setAisleCorners([]);
            setEditingAisle(newAisle);
        }
    }

    function handleAddManualAisle() {
        const nextIndex = aisles.length + 1;
        setEditingAisle({
            id: `aisle_${nextIndex}`,
            segment_id: `aisle_${nextIndex}`,
            name: `Aisle ${nextIndex}`,
            x_min: 0.0,
            x_max: 2.5,
            y_min: 0.0,
            y_max: 2.0,
            is_single_lane: true,
            isNew: true,
        });
    }

    function handleSaveAisle(savedAisle) {
        const targetId = savedAisle.segment_id || savedAisle.id;
        const exists = aisles.some((a) => (a.segment_id || a.id) === targetId);
        let updated;
        if (exists) {
            updated = aisles.map((a) => ((a.segment_id || a.id) === targetId ? savedAisle : a));
        } else {
            updated = [...aisles, savedAisle];
        }
        const normalized = normalizeAisles(updated);
        setAisles(normalized);
        saveAislesConfig(normalized);
        setEditingAisle(null);
        setSelectedAisleId(targetId);
        setToastMessage(`Saved aisle "${savedAisle.name}" and broadcasted to fleet!`);
    }

    function handleDeleteAisle(segId) {
        const updated = aisles.filter((a) => (a.segment_id || a.id) !== segId);
        setAisles(updated);
        saveAislesConfig(updated);
        if (selectedAisleId === segId) setSelectedAisleId(null);
        setToastMessage(`Deleted aisle ${segId} from fleet.`);
    }

    function handleResetDefaults() {
        setAisles(DEFAULT_AISLES);
        saveAislesConfig(DEFAULT_AISLES);
        setToastMessage("Reset aisles to 3 default warehouse corridors.");
    }

    function handleSaveFleetAisles() {
        saveAislesConfig(aisles);
        setToastMessage(`Saved & broadcasted ${aisles.length} aisles to all fleet robots!`);
    }

    function handleSelectAisleFromMap(aisle) {
        setSelectedAisleId(aisle.segment_id || aisle.id);
        setAisleDrawerOpen(true);
    }

    // ── Staged Tasks state (Draft tasks ready for batch broadcast) ─────────
    const [stagedTasks, setStagedTasks] = useState([]);
    const [selectionMode, setSelectionMode] = useState(false);
    const [selectedPoints, setSelectedPoints] = useState([]);
    const [mapZoom, setMapZoom] = useState({ zoom: 100, pixelsPerMeter: 20 });
    const [taskToDelete, setTaskToDelete] = useState(null);
    const [toastMessage, setToastMessage] = useState("");

    useEffect(() => {
        if (!toastMessage) return undefined;
        const timer = setTimeout(() => setToastMessage(""), 4500);
        return () => clearTimeout(timer);
    }, [toastMessage]);

    function startTaskSelection() {
        setAisleDrawingMode(false);
        setAisleCorners([]);
        setSelectedPoints([]);
        setSelectionMode(true);
    }

    function cancelTaskSelection() {
        setSelectedPoints([]);
        setSelectionMode(false);
    }

    function requestDeleteTask(taskNumber) {
        setTaskToDelete(tasks.find((task) => task.number === taskNumber) || null);
    }

    function confirmDeleteTask() {
        if (!taskToDelete) return;
        setTasks((current) => current.filter((task) => task.number !== taskToDelete.number));
        setTaskToDelete(null);
    }

    // Handles map click points for staging tasks (point 0: pickup, point 1: dropoff)
    function handlePointSelect(point) {
        const points = [...selectedPoints, point];
        setSelectedPoints(points);
        if (points.length === 2) {
            const nextIndex = stagedTasks.length + 1;
            const pRos = svgToRos(points[0].x, points[0].y);
            const dRos = svgToRos(points[1].x, points[1].y);
            const newStaged = {
                id: `staged-${Date.now()}-${nextIndex}`,
                number: nextIndex,
                name: `T${nextIndex}`,
                start: points[0],
                end: points[1],
                pickupRos: pRos,
                dropoffRos: dRos,
                priority: 1,
            };
            setStagedTasks((prev) => [...prev, newStaged]);
            setSelectedPoints([]);

            if (stagedTasks.length + 1 >= 5) {
                setSelectionMode(false);
                setToastMessage("Staged 5 tasks! Review them and click 'Broadcast Fleet Tasks' to deploy.");
            } else {
                setToastMessage(`Staged Task T${nextIndex} (${stagedTasks.length + 1}/5). Click map to add next or Broadcast now.`);
            }
        }
    }

    function handleBroadcastStaged() {
        if (stagedTasks.length === 0) return;
        const sent = broadcastTasks(stagedTasks);
        if (sent) {
            setToastMessage(` Broadcasted ${stagedTasks.length} tasks to fleet as AVAILABLE!`);
            setStagedTasks([]);
            setSelectionMode(false);
        } else {
            // Local fallback when ROS bridge isn't running
            setTasks((current) => [
                ...current,
                ...stagedTasks.map((t) => ({
                    number: t.number,
                    name: t.name,
                    assigned: false,
                    robot: null,
                    progress: 0,
                    state: 0,
                    stateLabel: "Available",
                    start: t.start,
                    end: t.end,
                })),
            ]);
            setToastMessage(`Broadcasted ${stagedTasks.length} tasks locally (ROS offline fallback).`);
            setStagedTasks([]);
            setSelectionMode(false);
        }
    }

    function removeStagedTask(id) {
        setStagedTasks((prev) => prev.filter((t) => t.id !== id));
    }

    function clearStagedTasks() {
        setStagedTasks([]);
        setSelectedPoints([]);
        setSelectionMode(false);
    }

    return (
        <div
            className="flex h-screen flex-col overflow-hidden bg-[#f7f8f6] text-[#1b231f]"
            style={font}
        >
            <header className="relative flex min-h-14 flex-wrap items-center gap-3.5 border-b border-[#c4cbc5] bg-white px-3.5 py-3 md:h-14 md:flex-nowrap md:gap-5 md:px-5 md:py-0">
                <div
                    className="flex items-center gap-2.5 whitespace-nowrap text-[17px] font-bold"
                    style={displayFont}
                >
                    <span className="h-[9px] w-[9px] rounded-full bg-[#2f6f5e] shadow-[0_0_0_4px_#e4efe9]" />
                    ATLAS<i>FLEET</i>
                </div>
                <div className="hidden h-[22px] w-px bg-[#e3e6e1] md:block" />
                {/* ROS connection status badge */}
                <div className={`flex items-center gap-1.5 rounded-full px-2.5 py-1 text-[11px] font-semibold ${rosStatus === 'connected' ? 'bg-[#e4efe9] text-[#2f6f5e]'
                    : rosStatus === 'error' ? 'bg-[#f6e4e1] text-[#c0453b]'
                        : 'bg-[#eceee9] text-[#8e988f]'
                    }`}>
                    <span className={`h-[7px] w-[7px] rounded-full ${rosStatus === 'connected' ? 'bg-[#2f6f5e] animate-pulse'
                        : rosStatus === 'error' ? 'bg-[#c0453b]'
                            : 'bg-[#8e988f]'
                        }`} />
                    {rosStatus === 'connected' ? 'ROS Live' : rosStatus === 'error' ? 'ROS Error' : 'ROS Connecting…'}
                </div>

                {/* Single-lane Aisle Manager Toggle */}
                <button
                    onClick={() => setAisleDrawerOpen(true)}
                    className="flex items-center gap-1.5 rounded-full border border-[#6366f1]/35 bg-[#eef2ff] px-2.5 py-1 text-[11px] font-bold text-[#4338ca] hover:bg-[#e0e7ff] transition-colors"
                    title="View and configure single-lane aisle reservation zones"
                >
                    <span>🛣️</span>
                    <span>Aisles ({aisles.length})</span>
                </button>

                <div className="absolute left-1/2 -translate-x-1/2 whitespace-nowrap text-sm font-semibold" style={displayFont}>
                    LIVE FLOOR <i>MAP</i>
                </div>
                <div className="absolute right-3.5 hidden items-center gap-3.5 text-xs text-[#6b776f] lg:flex md:right-5">
                    <span className="flex items-center gap-1.5 whitespace-nowrap">
                        <i className="h-2 w-2 rounded-full bg-[#2f6f5e]" />
                        {robots.filter(r => r.online && r.state !== 'warn').length} Active
                    </span>
                    <span className="flex items-center gap-1.5 whitespace-nowrap">
                        <i className="h-2 w-2 rounded-full bg-[#c97a2b]" />
                        {robots.filter(r => r.online && r.battery <= 20).length} Low power
                    </span>
                    <span className="flex items-center gap-1.5 whitespace-nowrap">
                        <i className="h-2 w-2 rounded-full bg-[#b7beb8]" />
                        {robots.filter(r => !r.online).length} Offline
                    </span>
                </div>
            </header>
            <main className="flex min-h-0 flex-1 flex-col">
                <div className="flex min-h-0 flex-1 flex-col md:flex-row">
                    <RobotStatusPanel robots={robots} />
                    <MapPanel
                        robots={robots}
                        selectionMode={selectionMode}
                        selectedPoints={selectedPoints}
                        tasks={tasks}
                        stagedTasks={stagedTasks}
                        onPointSelect={handlePointSelect}
                        dockInfo={dockInfo}
                        aisles={aisles}
                        selectedAisleId={selectedAisleId}
                        onSelectAisle={handleSelectAisleFromMap}
                        aisleDrawingMode={aisleDrawingMode}
                        aisleCorners={aisleCorners}
                        onAisleCornerSelect={handleAisleCornerSelect}
                        onCancelAisleDraw={cancelAisleDraw}
                        onOpenAisleManager={() => setAisleDrawerOpen(true)}
                        trafficEvents={trafficEvents}
                        onZoomChange={setMapZoom}
                    />
                    <TaskStatusPanel
                        tasks={tasks}
                        stagedTasks={stagedTasks}
                        selectionMode={selectionMode}
                        onAdd={startTaskSelection}
                        onCancel={cancelTaskSelection}
                        onDelete={requestDeleteTask}
                        onBroadcastStaged={handleBroadcastStaged}
                        onRemoveStaged={removeStagedTask}
                        onClearStaged={clearStagedTasks}
                    />
                </div>
                <div className="flex min-h-0 flex-[0.95] flex-col md:flex-row">
                    <EfficiencyPanel tasks={tasks} mapZoom={mapZoom} />
                    <AllocationPanel allocationEvents={allocationEvents} trafficEvents={trafficEvents} liveBundles={liveBundles} />
                </div>
            </main>
            {taskToDelete && (
                <DeleteTaskDialog
                    task={taskToDelete}
                    onCancel={() => setTaskToDelete(null)}
                    onConfirm={confirmDeleteTask}
                />
            )}
            {aisleDrawerOpen && (
                <AisleManagerDrawer
                    isOpen={aisleDrawerOpen}
                    onClose={() => setAisleDrawerOpen(false)}
                    aisles={aisles}
                    selectedAisleId={selectedAisleId}
                    onSelectAisle={handleSelectAisleFromMap}
                    onStartDrawAisle={startAisleDraw}
                    onAddManualAisle={handleAddManualAisle}
                    onEditAisle={(a) => setEditingAisle(a)}
                    onDeleteAisle={handleDeleteAisle}
                    onResetDefaults={handleResetDefaults}
                    onSaveFleet={handleSaveFleetAisles}
                    trafficEvents={trafficEvents}
                    robots={robots}
                />
            )}
            {editingAisle && (
                <AisleEditModal
                    aisle={editingAisle}
                    isNew={editingAisle.isNew}
                    onClose={() => setEditingAisle(null)}
                    onSave={handleSaveAisle}
                />
            )}
            {toastMessage && (
                <div className="fixed bottom-6 left-1/2 -translate-x-1/2 z-50 flex items-center gap-2.5 rounded-full border border-[#2f6f5e]/40 bg-[#1b231f] text-white px-5 py-2.5 text-xs font-semibold shadow-2xl animate-[fadeIn_0.2s_ease-out]">
                    <span className="h-2 w-2 rounded-full bg-[#2f6f5e] animate-ping" />
                    <span>{toastMessage}</span>
                </div>
            )}
        </div>
    );
}

export default App;
