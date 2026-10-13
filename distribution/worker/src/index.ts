// Release distribution for the FlyByWire A380X X-Plane port (GPL-3.0).
//
//   GET /manifest                    the latest release: version, files, SHA-256
//   GET /download/<version>/<file>   a release file (a redirect to GitHub's download)
//
// Releases live on GitHub Releases of GITHUB_REPO, tagged v<version>, with
// manifest.json among their assets. The repository is public at release; an
// optional GITHUB_TOKEN secret lets the Worker read it while still private.
// The installer verifies every file against the manifest's SHA-256 before
// installing anything, so the redirect needs no trust.

const SAFE = /^[A-Za-z0-9._-]{1,128}$/;

interface Asset {
	name: string;
	url: string;
	size: number;
}

interface Release {
	tag_name: string;
	assets: Asset[];
}

type Config = Env & { GITHUB_TOKEN?: string };

function log(event: string, fields: Record<string, unknown>): void {
	console.log(JSON.stringify({ event, ...fields }));
}

function error(status: number, message: string): Response {
	return Response.json({ error: message }, { status });
}

function github(env: Config, path: string, accept = "application/vnd.github+json"): Promise<Response> {
	const headers: Record<string, string> = {
		accept,
		"user-agent": "fbw-a380x-xp-releases",
		"x-github-api-version": "2022-11-28",
	};
	if (env.GITHUB_TOKEN) {
		headers.authorization = `Bearer ${env.GITHUB_TOKEN}`;
	}
	const url = path.startsWith("https://") ? path : `https://api.github.com/repos/${env.GITHUB_REPO}${path}`;
	// Asset downloads answer with a redirect to a signed URL: returned, not
	// followed, so a token never goes to the storage host.
	return fetch(url, { headers, redirect: "manual" });
}

async function release(env: Config, which: string): Promise<Release | null> {
	const r = await github(env, which === "latest" ? "/releases/latest" : `/releases/tags/v${which}`);
	if (r.status === 404) {
		return null;
	}
	if (!r.ok) {
		throw new Error(`GitHub ${r.status} for release ${which}`);
	}
	return (await r.json()) as Release;
}

// The asset's signed download URL (valid a few minutes).
async function signedUrl(env: Config, asset: Asset): Promise<string> {
	const r = await github(env, asset.url, "application/octet-stream");
	const location = r.headers.get("location");
	if (r.status >= 300 && r.status < 400 && location) {
		return location;
	}
	throw new Error(`GitHub ${r.status} for asset ${asset.name}`);
}

export default {
	async fetch(request, env: Config): Promise<Response> {
		const url = new URL(request.url);
		if (request.method !== "GET" && request.method !== "HEAD") {
			return error(405, "method not allowed");
		}
		try {
			if (url.pathname === "/manifest") {
				const latest = await release(env, "latest");
				const asset = latest?.assets.find((a) => a.name === "manifest.json");
				if (!latest || !asset) {
					return error(404, "no release published yet");
				}
				const body = await fetch(await signedUrl(env, asset));
				if (!body.ok) {
					throw new Error(`manifest download ${body.status}`);
				}
				log("manifest", { tag: latest.tag_name });
				return new Response(body.body, {
					headers: {
						"content-type": "application/json",
						// Short: a new release should reach installers quickly.
						"cache-control": "public, max-age=60",
					},
				});
			}

			const parts = url.pathname.split("/").filter((p) => p.length > 0);
			if (parts.length === 3 && parts[0] === "download") {
				const [, version, file] = parts;
				if (!SAFE.test(version) || !SAFE.test(file) || version.startsWith(".") || file.startsWith(".")) {
					return error(400, "bad release path");
				}
				const rel = await release(env, version);
				const asset = rel?.assets.find((a) => a.name === file);
				if (!asset) {
					return error(404, "no such release file");
				}
				log("download", { version, file, size: asset.size });
				return Response.redirect(await signedUrl(env, asset), 302);
			}

			return error(404, "not found");
		} catch (e) {
			log("error", { path: url.pathname, message: e instanceof Error ? e.message : String(e) });
			return error(502, "release server could not reach GitHub");
		}
	},
} satisfies ExportedHandler<Config>;
