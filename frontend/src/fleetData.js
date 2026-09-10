export const robots = [
    { id: 'R-01', x: 120, y: 480, state: 'active', task: 'Pallet move — Bay 3', battery: 81, online: true },
    { id: 'R-02', x: 420, y: 180, state: 'active', task: 'Restock — Aisle 12', battery: 64, online: true },
    { id: 'R-03', x: 640, y: 520, state: 'active', task: 'Return to dock', battery: 47, online: true },
    { id: 'R-04', x: 520, y: 380, state: 'active', task: 'Pallet move — Bay 7', battery: 73, online: true },
    { id: 'R-05', x: 760, y: 220, state: 'warn', task: 'Restock — Aisle 4', battery: 18, online: true },
    { id: 'R-06', x: 340, y: 250, state: 'active', task: 'Idle at charger', battery: 96, online: true },
    { id: 'R-07', x: 200, y: 150, state: 'offline', task: '—', battery: 0, online: false },
    { id: 'R-08', x: 830, y: 470, state: 'active', task: 'Inspection sweep — Zone D', battery: 58, online: true },
]

export const initialTasks = [
    { name: 'Pallet move — Bay 3', assigned: true, robot: 'R-01', progress: 62 },
    { name: 'Restock — Aisle 12', assigned: true, robot: 'R-02', progress: 28 },
    { name: 'Inspection sweep — Zone D', assigned: true, robot: 'R-08', progress: 80 },
    { name: 'Return empty carts — Dock 2', assigned: false, robot: null, progress: 0 },
]

export const ganttRows = [
    { label: 'R-01', bars: [{ start: 0, end: 90, progress: 100, status: 'done' }] },
    { label: 'R-02', bars: [{ start: 60, end: 180, progress: 55, status: 'active' }] },
    { label: 'R-08', bars: [{ start: 30, end: 120, progress: 100, status: 'done' }] },
    { label: 'R-04', bars: [{ start: 120, end: 240, progress: 40, status: 'active' }] },
    { label: 'R-03', bars: [{ start: 210, end: 255, progress: 70, status: 'active' }] },
    { label: 'R-05', bars: [{ start: 240, end: 330, progress: 20, status: 'delayed' }] },
    { label: 'Unassigned', bars: [{ start: 300, end: 360, progress: 0, status: 'scheduled' }] },
]

export const mapPaths = [
    { d: 'M 120 480 L 260 480 L 260 320 L 420 320 L 420 180' },
    { d: 'M 640 520 L 640 380 L 520 380' },
    { d: 'M 760 220 L 760 340 L 660 340', warn: true },
    { d: 'M 200 150 L 340 150 L 340 250' },
]
