export const robots = [
    { id: 'R-01', x: 120, y: 480, state: 'active', task: 'Pallet move — Bay 3', battery: 81, online: true, color: '#3978b7', colorDim: '#e4edf7' },
    { id: 'R-02', x: 420, y: 180, state: 'active', task: 'Restock — Aisle 12', battery: 64, online: true, color: '#7b61a8', colorDim: '#eee9f6' },
    { id: 'R-03', x: 640, y: 520, state: 'active', task: 'Return to dock', battery: 47, online: true, color: '#c98a2e', colorDim: '#f7eddb' },
    { id: 'R-04', x: 520, y: 380, state: 'active', task: 'Pallet move — Bay 7', battery: 73, online: true, color: '#2b9aa0', colorDim: '#e1f1f2' },
]

export const initialTasks = [
    { number: 1, name: 'Task 1', assigned: false, robot: null, progress: 0, start: { x: 117, y: 158 }, end: { x: 319, y: 433 } },
    { number: 2, name: 'Task 2', assigned: false, robot: null, progress: 0, start: { x: 690, y: 125 }, end: { x: 806, y: 462 } },
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
