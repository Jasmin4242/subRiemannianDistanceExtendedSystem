function YiYjyk_p =  get_YiYjyk_p(vektorfeld_rechts,vektorfeld_links,koordinate_nummer,y,p,n)

    %zuerst zB Y2y4 bestimmen
    ergebnis_schritt1=vektorfeld_rechts(koordinate_nummer);
    
    %dann Y1Y2y4 bestimmen
    sym ergebnis_schritt2;
    ergebnis_schritt2=0;
    for ii=1:n
        ergebnis_schritt2=ergebnis_schritt2+vektorfeld_links(ii)*diff(ergebnis_schritt1,y(ii));
    end
    YiYjyk_p=subs(ergebnis_schritt2,[y(1) y(2) y(3) y(4)],[p(1) p(2) p(3) p(4)]);

end

