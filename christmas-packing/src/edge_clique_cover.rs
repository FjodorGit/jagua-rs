use std::collections::HashSet;

pub fn greedy_edge_clique_cover(n: usize, edges: &[(u32, u32)]) -> Vec<Vec<usize>> {
    // Adjacency set for quick lookup
    let mut adj: Vec<HashSet<usize>> = vec![HashSet::new(); n];
    for &(u, v) in edges {
        adj[u as usize].insert(v as usize);
        adj[v as usize].insert(u as usize);
    }

    // Track uncovered edges
    let mut uncovered: HashSet<(usize, usize)> = edges
        .iter()
        .map(|&(u, v)| (u as usize, v as usize))
        .collect();

    let mut cliques: Vec<Vec<usize>> = Vec::new();

    while !uncovered.is_empty() {
        // Pick an uncovered edge
        let &(u, v) = uncovered.iter().next().unwrap();

        // Greedily extend to a clique starting from {u, v}
        let mut clique: Vec<usize> = vec![u, v];
        let mut candidates: Vec<usize> = adj[u].intersection(&adj[v]).copied().collect();

        while let Some(w) = candidates.pop() {
            // Check if w is connected to all current clique members
            if clique.iter().all(|&c| adj[w].contains(&c)) {
                clique.push(w);
                // Update candidates: must be connected to w as well
                candidates.retain(|&c| adj[w].contains(&c));
            }
        }

        // Mark all edges in this clique as covered
        for i in 0..clique.len() {
            for j in (i + 1)..clique.len() {
                let (a, b) = (clique[i], clique[j]);
                uncovered.remove(&(a, b));
                uncovered.remove(&(b, a));
            }
        }

        cliques.push(clique);
    }

    cliques
}
