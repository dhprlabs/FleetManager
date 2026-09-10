export const robots = [
    { id: 'robot1', x: 111, y: 392, angleDeg: -60, state: 'active', task: 'T1 — Pallet transit', battery: 94, online: true, color: '#3978b7', colorDim: '#e4edf7' },
    { id: 'robot2', x: 140, y: 406, angleDeg: 0, state: 'active', task: 'T2 — Restock', battery: 88, online: true, color: '#7b61a8', colorDim: '#eee9f6' },
    { id: 'robot3', x: 195, y: 371, angleDeg: 30, state: 'active', task: 'T3 — Bay transit', battery: 72, online: true, color: '#c98a2e', colorDim: '#f7eddb' },
    { id: 'robot4', x: 260, y: 360, angleDeg: -90, state: 'idle', task: 'Standby', battery: 100, online: false, color: '#2b9aa0', colorDim: '#e1f1f2' },
]

export const initialTasks = [
    { number: 1, name: 'T1', assigned: true, robot: 'robot1', progress: 45, start: { x: 156, y: 319 }, end: { x: 76, y: 359 } },
    { number: 2, name: 'T2', assigned: true, robot: 'robot2', progress: 20, start: { x: 186, y: 319 }, end: { x: 46, y: 359 } },
]

export const ganttRows = [
    { label: 'robot1', bars: [{ start: 0, end: 90, progress: 100, status: 'done' }] },
    { label: 'robot2', bars: [{ start: 60, end: 180, progress: 55, status: 'active' }] },
    { label: 'robot3', bars: [{ start: 30, end: 120, progress: 100, status: 'done' }] },
    { label: 'robot4', bars: [{ start: 120, end: 240, progress: 0, status: 'scheduled' }] },
]

export const mapPaths = []
