export interface GithubRelease {
  name: string;
  publishedAt: Date;
  htmlUrl: string;
  body: string;
}

interface GithubApiRelease {
  name: string | null;
  tag_name: string;
  published_at: string;
  html_url: string;
  body: string | null;
  draft: boolean;
  prerelease: boolean;
}

export const getGithubReleases = async (
  owner: string,
  repo: string,
  includePreReleases = false,
  skip = 0,
  take = 5,
): Promise<GithubRelease[]> => {
  const perPage = Math.max(take, 1);
  const page = Math.floor(skip / perPage) + 1;
  const response = await fetch(
    `https://api.github.com/repos/${owner}/${repo}/releases?per_page=${perPage}&page=${page}`,
    {
      headers: { Accept: 'application/vnd.github+json' },
    },
  );

  if (!response.ok) {
    throw new Error(`GitHub releases for ${owner}/${repo}: ${response.status}`);
  }

  const releases = (await response.json()) as GithubApiRelease[];

  return releases
    .filter((release) => !release.draft && (includePreReleases || !release.prerelease))
    .map((release) => ({
      name: release.name || release.tag_name,
      publishedAt: new Date(release.published_at),
      htmlUrl: release.html_url,
      body: release.body ?? '',
    }));
};
