import { parseAnalysisDataset, type FlightAnalysisDataset } from "./analysis-data.ts";

const databaseName = "birdman-flight-analysis";
const storeName = "flights";

/** Browser adapter: transactions commit before the analysis tab navigates. */
async function openDatabase(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open(databaseName, 1);
    request.onupgradeneeded = () => request.result.createObjectStore(storeName);
    request.onerror = () => reject(new Error(`Analysis database unavailable: ${request.error?.message ?? "unknown error"}`));
    request.onblocked = () => reject(new Error("Analysis database blocked by another tab"));
    request.onsuccess = () => {
      const database = request.result;
      database.onversionchange = () => database.close();
      resolve(database);
    };
  });
}

export async function storeAnalysis(id: string, dataset: FlightAnalysisDataset): Promise<void> {
  const database = await openDatabase();
  try {
    await new Promise<void>((resolve, reject) => {
      const transaction = database.transaction(storeName, "readwrite");
      transaction.oncomplete = () => resolve();
      transaction.onabort = () => reject(new Error(`Analysis save failed: ${transaction.error?.message ?? "transaction aborted"}`));
      transaction.objectStore(storeName).add(dataset, id);
    });
  } finally { database.close(); }
}

export async function loadAnalysis(id: string): Promise<FlightAnalysisDataset> {
  const database = await openDatabase();
  try {
    const value: unknown = await new Promise((resolve, reject) => {
      const transaction = database.transaction(storeName, "readonly");
      const request = transaction.objectStore(storeName).get(id);
      request.onsuccess = () => resolve(request.result);
      request.onerror = () => reject(new Error("Analysis read failed"));
    });
    if (value === undefined) throw new Error("Flight not found; open analysis from the simulator again");
    return parseAnalysisDataset(JSON.stringify(value));
  } finally { database.close(); }
}
